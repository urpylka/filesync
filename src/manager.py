#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:


from json_database import JSONDatabase
import os.path
import rospy
import time
from mavros_msgs.msg import State
from mavros_msgs.srv import FileList
from threading import Thread, Lock
import requests
from mavros.utils import *
from mavros.nuttx_crc32 import *
import ftp # /opt/ros/kinetic/lib/python2.7/dist-packages/mavros/ftp.py


stop_operation = False

def arming_protect(data):
    if data.armed and ARMING_PROTECT:
        stop_operation = True
    else:
        stop_operation = False


def convert_to_rpi_path(log_directory, path_on_px4):
    return log_directory + '/' + os.path.basename(os.path.dirname(path_on_px4)).replace('-','') + '_' + os.path.basename(path_on_px4).replace('_','')


def downloader(db, _LOGS_DIRECTORY, _CHECK_FILE_CHECKSUM, mavftp_lock):
    print("Создан downloader")
    """
    По-хорошему, нужна проверка что лог уже не скачен, и имя не повторяется
    
    rosrun mavros mavftp download /fs/microsd/log/2017-10-20/13_29_53.ulg 13_29_53.ulg
    
    rosservice call /mavros/ftp/open /fs/microsd/log/2017-10-20/10_53_02.ulg 0
    size: 1676016
    success: True
    r_errno: 0
    
    rosservice call /mavros/ftp/read /fs/microsd/log/2017-10-20/10_53_02.ulg 0 1676015
    data: []
    success: False
    r_errno: 5
    
    Ошибка 9 возникает когда файл не открыт
    
    rosservice call /mavros/ftp/read /fs/microsd/log/2017-10-20/10_53_02.ulg 0 100
    data: [85, 76, 111, 103, 1, 18, 53, 1, 58, 201, 155, 24, 0, 0, 0, 0, 40, 0, 66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 0, 73, 15, 99, 104, 97, 114, 91, 52, 48, 93, 32, 118, 101, 114, 95, 115, 119, 53, 56, 50, 48, 98, 102, 98, 55, 57, 52, 48, 101, 99, 54, 57, 53, 55, 51, 99, 97, 101, 51]
    success: True
    r_errno: 0
    
    Кароче качать можно и нужно вообще побайтно (в принципе как я понял можно уже в полете), но туповато это самому реализовывать поэтому возьму это из mavftp
    """
    while True:
        log = db.dq.get()

        while not log['downloaded']:
            path_on_rpi = convert_to_rpi_path(_LOGS_DIRECTORY, log['path_on_px4'])
            try:
                with mavftp_lock:
                    time.sleep(1)
                    download(log['path_on_px4'], path_on_rpi, True, True, not _CHECK_FILE_CHECKSUM)
                # если успешно донладим, меняем флаг в словаре в памяти
                db.on_download(log, path_on_rpi)
            except Exception as ex:
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                print("Downloader error: " + str(ex))
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE" & сбросить ftp "rosservice call /mavros/ftp/reset"    
                time.sleep(2)
                # вообще можно перейти к другому элементу из очереди


def uploader(db, _USER_EMAIL, _USER_FEEDBACK, _UPLOADER_URL):
    print("Создан uploader")
    while True: #тк демон
        log = db.uq.get()

        while not log['uploaded']:
            try:
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



ftp_list_call = rospy.ServiceProxy('/mavros/ftp/list', FileList)


def finder(db, _FINDER_INTERVAL, mavftp_lock):
    print("Создан finder")
    """
    Функция поиска новых логов на PX4.

    Тк пикс не создает супервложенных директорий,
    можно обойтись без рекурсивного перехода между папками
    """
    while True:
        try:
            with mavftp_lock:
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


def create_downloaders(db, count, _LOGS_DIRECTORY, _CHECK_FILE_CHECKSUM, mavftp_lock):
    for i in range(count):
      t = Thread(target = downloader, args = (db, _LOGS_DIRECTORY, _CHECK_FILE_CHECKSUM, mavftp_lock, ))
      t.daemon = True
      t.start()


def create_uploaders(db, count, _USER_EMAIL, _USER_FEEDBACK, _UPLOADER_URL):
    for i in range(count):
      t = Thread(target = uploader, args = (db, _USER_EMAIL, _USER_FEEDBACK, _UPLOADER_URL, ))
      t.daemon = True
      t.start()


def create_finder(db, _FINDER_INTERVAL, mavftp_lock):
    t = Thread(target = finder, args = (db, _FINDER_INTERVAL, mavftp_lock, ))
    t.daemon = True
    t.start()


# AttributeError: __exit__
# mavftp.do_download_local("/fs/microsd/log/2017-10-20/10_53_02.ulg","logs/10_53_02.ulg",True,True,True)


def download(file_path,file_name,verbose=True,no_progressbar=False,no_verify=False):
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
                #if stop_operation: raise Exception("ARMING: Скачивание лога " + file_path + " приостановлено")
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

    except rospy.ServiceException as ex:
        print("Download error: " + str(ex))


def upload(description, additional_feedback, file_path, email, uploader_url, allow_for_analysis = 'false', obfuscated = 'false'):

    #print('description: ' + description)
    #print('additional_feedback: ' + additional_feedback)
    #print('email: ' + email)
    #print('uploader_url: ' + uploader_url)
    #print('file_path: ' + file_path)
    #print('allow_for_analysis: ' + allow_for_analysis)
    #print('obfuscated: ' + obfuscated)

    files = {'filearg': open(file_path,'rb')}
    values = {'description': description, 'feedback': additional_feedback, 'email': email, 'allowForAnalysis': allow_for_analysis, 'obfuscated': obfuscated}
    r = requests.post(uploader_url, files = files, data = values)
    
    #print('server_response_status_code: ' + str(r.status_code))
    #print('log\'s url: ' + r.url)

    return r.status_code, r.url
    #if stop_operation: raise Exception("ARMING: Скачивание лога " + file_path + " приостановлено")

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
    
    rospy.Subscriber('/mavros/state', State, arming_protect)
    mavftp_lock = Lock()
    
    db = JSONDatabase(DB_JSON_PATH)
    create_downloaders(db, DOWNLOADERS_COUNT, LOGS_DIRECTORY, CHECK_FILE_CHECKSUM, mavftp_lock)
    create_uploaders(db, UPLOADERS_COUNT, USER_EMAIL, USER_FEEDBACK, UPLOADER_URL)
    create_finder(db, FINDER_INTERVAL, mavftp_lock)
    
    rospy.spin()
    #db.dq.join()
    #db.uq.join()

if __name__ == '__main__':
    main()
