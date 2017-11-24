#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import os.path, rospy, time, requests
from json_database import JSONDatabase
from mavros_msgs.msg import State
from mavros_msgs.srv import FileList
from threading import Thread, Lock, Event
from mavros.utils import *
from mavros.nuttx_crc32 import *
from mavros import ftp # /opt/ros/kinetic/lib/python2.7/dist-packages/mavros/ftp.py


def convert_to_rpi_path(log_directory, path_on_px4):
    return log_directory + '/' + os.path.basename(os.path.dirname(path_on_px4)).replace('-','') + '_' + os.path.basename(path_on_px4).replace('_','')


def downloader(db, _LOGS_DIRECTORY, _CHECK_FILE_CHECKSUM, mavftp_lock, accept_operation):
    print("Created downloader")
    """
    По-хорошему, нужна проверка что лог уже не скачен, и имя не повторяется
    https://gist.github.com/urpylka/0b0c2dcd55141ae1735df00b78f713fb

    Кароче качать можно и нужно вообще побайтно (в принципе как я понял можно уже в полете), но туповато это самому реализовывать поэтому возьму это из mavftp
    """
    while True:
        log = db.dq.get()

        while not log['downloaded']:
            path_on_rpi = convert_to_rpi_path(_LOGS_DIRECTORY, log['path_on_px4'])
            try:
                with mavftp_lock:
                    time.sleep(1)
                    download(log['path_on_px4'], path_on_rpi, accept_operation, True, True, not _CHECK_FILE_CHECKSUM)
                # если успешно донладим, меняем флаг в словаре в памяти
                db.on_download(log, path_on_rpi)
            except Exception as ex:
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                print("Downloader error: " + str(ex))
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE" & сбросить ftp "rosservice call /mavros/ftp/reset"    
                time.sleep(2)
                # вообще можно перейти к другому элементу из очереди


def uploader(db, _USER_EMAIL, _USER_FEEDBACK, _UPLOADER_URL, accept_operation):
    print("Created uploader")
    while True: #тк демон
        log = db.uq.get()

        while not log['uploaded']:
            try:

                accept_operation.wait()

                # хорошая идея, если развертывать свой logs.px4.io можно возвращаться как-то еще ссылку на удаление лога
                status_code, log_url = upload(os.path.basename(log['path_on_rpi']), _USER_FEEDBACK, log['path_on_rpi'], _USER_EMAIL, _UPLOADER_URL)
                if status_code == 200:
                    db.on_upload(log, log_url)
            except Exception as ex:
                # мб такая ошибка [Errno 2] No such file or directory: u'logs/20171019_143151.ulg'
                # в этом случае надо сбросить log[download] = False
                print("Uploader error: " + str(ex))
                time.sleep(2)
                # вообще можно перейти к другому элементу из очереди


def finder(db, _FINDER_INTERVAL, mavftp_lock, accept_operation, ftp_list_call):
    print("Created finder")
    """
    Функция поиска новых логов на PX4.

    Тк пикс не создает супервложенных директорий,
    можно обойтись без рекурсивного перехода между папками
    """
    while True:
        try:
            with mavftp_lock:

                accept_operation.wait()

                time.sleep(1) # чтобы поток успел умереть
                print("Ищу новые логи...")
                logs_folder = ftp_list_call("/fs/microsd/log")
                if logs_folder.success and logs_folder.r_errno == 0:

                    # проход по папкам с файлами логов
                    for logs_folder_list in logs_folder.list:

                        # если type равен 0, значит это файл, если 1, то это папка
                        if logs_folder_list.type == 1:

                            path_session_logs_folder = "/fs/microsd/log/" + logs_folder_list.name

                            session_logs_folder = ftp_list_call(path_session_logs_folder)
                            if session_logs_folder.success and session_logs_folder.r_errno == 0:

                                for log in session_logs_folder.list:
                                    if (log.type == 0):
                                        # если найден файл лога
                                        log_path = path_session_logs_folder + "/" + log.name
                                        if not db.is_log_in_records(log_path): db.on_find(log_path)

                            else: print("Import log folder error in " + path_session_logs_folder)
                else: print("Import log folder error in /fs/microsd/log")
        except Exception as ex:
            print("Finder error: " + str(ex))
            #time.sleep(2)
            #continue
        time.sleep(_FINDER_INTERVAL)


def create_downloaders(db, count, _LOGS_DIRECTORY, _CHECK_FILE_CHECKSUM, mavftp_lock, accept_operation):
    for i in range(count):
      t = Thread(target = downloader, args = (db, _LOGS_DIRECTORY, _CHECK_FILE_CHECKSUM, mavftp_lock, accept_operation, ))
      t.daemon = True
      t.start()


def create_uploaders(db, count, _USER_EMAIL, _USER_FEEDBACK, _UPLOADER_URL, accept_operation):
    for i in range(count):
      t = Thread(target = uploader, args = (db, _USER_EMAIL, _USER_FEEDBACK, _UPLOADER_URL, accept_operation, ))
      t.daemon = True
      t.start()


def create_finder(db, _FINDER_INTERVAL, mavftp_lock, accept_operation, ftp_list_call):
    t = Thread(target = finder, args = (db, _FINDER_INTERVAL, mavftp_lock, accept_operation, ftp_list_call, ))
    t.daemon = True
    t.start()


# AttributeError: __exit__
# mavftp.do_download_local("/fs/microsd/log/2017-10-20/10_53_02.ulg","logs/10_53_02.ulg",True,True,True)


def download(file_path, file_name, accept_operation, verbose=True, no_progressbar=False, no_verify=False):
    # optimized transfer size for FTP message payload
    # XXX: bug in ftp.cpp cause a doubling request of last package.
    # -1 fixes that.
    FTP_CHUNK = 239 * 18 - 1

    mavros.set_namespace("/mavros")

    local_crc = 0
    local_file = open(file_name, 'wb')

    print_if(verbose, "Downloading from", file_path, "to", file_name)
    # https://stackoverflow.com/questions/7447284/how-to-troubleshoot-an-attributeerror-exit-in-multiproccesing-in-python

    try:
        #remote_file = ftp.open(file_path, 'r')
        with ftp.open(file_path, 'r') as remote_file:
            while True:

                accept_operation.wait()

                buf = remote_file.read(FTP_CHUNK)
                if len(buf) == 0:
                    break

                local_crc = nuttx_crc32(buf, local_crc)
                local_file.write(buf)

                if not no_verify:
                    print_if(verbose, "Verifying...")
                    remote_crc = ftp.checksum(file_path)
                    if local_crc != remote_crc:
                        fault("Verification failed: 0x{local_crc:08x} != 0x{remote_crc:08x}".format(**locals()))

    except Exception as ex:
        print("Download error: " + str(ex))
        ftp.reset_server()


def upload(description, additional_feedback, file_path, email, uploader_url, allow_for_analysis = 'false', obfuscated = 'false'):
	
    files = {'filearg': open(file_path,'rb')}
    values = {'description': description, 'feedback': additional_feedback, 'email': email, 'allowForAnalysis': allow_for_analysis, 'obfuscated': obfuscated}
    r = requests.post(uploader_url, files = files, data = values)

    return r.status_code, r.url


def main():
    rospy.init_node('px4logs_manager')

    USER_EMAIL = rospy.get_param('~user_email')
    USER_FEEDBACK = rospy.get_param('~user_feedback') #"Error: Please enter additional_feedback for your logs"
    FINDER_INTERVAL = rospy.get_param('~finder_interval', 10)
    ARMING_PROTECT = rospy.get_param('~arming_protect', False)
    DOWNLOADERS_COUNT = rospy.get_param('~donwloaders_count', 1) # [Errno 24] Too many open files
    UPLOADERS_COUNT = rospy.get_param('~uploaders_count', 2)
    CHECK_FILE_CHECKSUM = rospy.get_param('~check_file_checksum', False)
    LOGS_DIRECTORY = rospy.get_param('~logs_directory')
    DB_JSON_PATH = rospy.get_param('~db_json_path')
    UPLOADER_URL = rospy.get_param('~uploader_url', "https://logs.px4.io/upload")

    rospy.loginfo('Inited px4logs_manager')

    ftp_list_call = rospy.ServiceProxy('/mavros/ftp/list', FileList)

    accept_operation = Event()
    accept_operation.set()

    def arming_protect(data):
        if data.armed and ARMING_PROTECT:
            if accept_operation.is_set():
                accept_operation.clear()
                print("ARMING все операции кроме upload заблокированы")
        else:
            if not accept_operation.is_set():
                accept_operation.set()
                print("DISARMING все операции разблокированы")


    rospy.Subscriber('/mavros/state', State, arming_protect)
    mavftp_lock = Lock()

    db = JSONDatabase(DB_JSON_PATH)
    create_downloaders(db, DOWNLOADERS_COUNT, LOGS_DIRECTORY, CHECK_FILE_CHECKSUM, mavftp_lock, accept_operation)
    create_uploaders(db, UPLOADERS_COUNT, USER_EMAIL, USER_FEEDBACK, UPLOADER_URL, accept_operation)
    create_finder(db, FINDER_INTERVAL, mavftp_lock, accept_operation, ftp_list_call)

    rospy.spin()
    #db.dq.join()
    #db.uq.join()

if __name__ == '__main__':
    main()
