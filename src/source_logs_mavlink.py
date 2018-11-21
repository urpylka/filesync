#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

# Copyright 2017-2018 Smirnov Artem.

import rospy, time, requests, mavros, os
from json_database import JSONDatabase
from mavros_msgs.msg import State
from mavros_msgs.srv import FileList
from threading import Thread, Lock, Event
from mavros.utils import *
from mavros.nuttx_crc32 import *
from mavros import ftp # /opt/ros/kinetic/lib/python2.7/dist-packages/mavros/ftp.py
from __future__ import print_function

def _resolve_path(path = None):
    """
    Resolve FTP path using PWD file
    """
    if os.path.exists(FTP_PWD_FILE):
        with open(FTP_PWD_FILE, 'r') as fd:
            pwd = fd.readline()
    else:
        # default home location is root directory
        pwd = os.environ.get('MAVFTP_HOME', '/')

    if not path:
        return os.path.normpath(pwd)    # no path - PWD location
    elif path.startswith('/'):
        return os.path.normpath(path)   # absolute path
    else:
        return os.path.normpath(os.path.join(pwd, path))


class ProgressBar:
    """
    Wrapper class for hiding file transfer progressbar construction
    """
    def __init__(self, quiet, operation, maxval):
        if no_progressbar or quiet or maxval == 0:
            print_if(maxval == 0, "Can't show progressbar for unknown file size", file=sys.stderr)
            self.pbar = None
            return

        self.pbar = pbar.ProgressBar(
            widgets=[operation, pbar.Percentage(), ' ', pbar.Bar(), ' ', pbar.ETA(), ' ', pbar.FileTransferSpeed()],
            maxval=maxval).start()

    def update(self, value):
        if self.pbar:
            self.pbar.update(value)

    def __enter__(self):
        if self.pbar:
            self.pbar.start()

        return self

    def __exit__(self, type, value, traceback):
        if self.pbar:
            self.pbar.finish()


def do_download_local(file_path,file_name,verbose=True,no_progressbar=False,no_verify=False):
    """
    AttributeError: __exit__
    mavftp.do_download_local("/fs/microsd/log/2017-10-20/10_53_02.ulg","logs/10_53_02.ulg",True,True,True)

    ftp_download_ulog.do_download_local(_log['path_on_px4'],_log['path_on_rpi'],True,True,True)

    Posible errors:

    донладим (по хорошему нужна проверка что лог уже не скачен и имя не повторяется)
    rosrun mavros mavftp download /fs/microsd/log/2017-10-20/13_29_53.ulg 13_29_53.ulg
    
    pi@raspberrypi:~/urpylka_px4logs $ rosservice call /mavros/ftp/open /fs/microsd/log/2017-10-20/10_53_02.ulg 0
    size: 1676016
    success: True
    r_errno: 0
    
    pi@raspberrypi:~/urpylka_px4logs $ rosservice call /mavros/ftp/read /fs/microsd/log/2017-10-20/10_53_02.ulg 0 1676015
    data: []
    success: False
    r_errno: 5
    
    ошибка 9 возникает когда файл не открыт
    
    pi@raspberrypi:~/urpylka_px4logs $ rosservice call /mavros/ftp/read /fs/microsd/log/2017-10-20/10_53_02.ulg 0 100
    data: [85, 76, 111, 103, 1, 18, 53, 1, 58, 201, 155, 24, 0, 0, 0, 0, 40, 0, 66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 0, 73, 15, 99, 104, 97, 114, 91, 52, 48, 93, 32, 118, 101, 114, 95, 115, 119, 53, 56, 50, 48, 98, 102, 98, 55, 57, 52, 48, 101, 99, 54, 57, 53, 55, 51, 99, 97, 101, 51]
    success: True
    r_errno: 0
    
    кароче качать можно и нужно вообще побайтно (в принципе как я понял можно уже в полете), но туповато это самому реализовывать поэтому возьму это из mavftp

    """

    rospy.init_node("mavftp", anonymous=True)
    mavros.set_namespace("/mavros")
    
    local_crc = 0
    
    file_path = _resolve_path(file_path)
    file = open(file_name, 'wb')

    print_if(verbose, "Downloading from", file_path, "to", file_name, file=sys.stderr)
    # https://stackoverflow.com/questions/7447284/how-to-troubleshoot-an-attributeerror-exit-in-multiproccesing-in-python
    to_fd = file
    from_fd = ftp.open(file_path, 'r')
    bar = ProgressBar(no_progressbar, "Downloading: ", from_fd.size)
    
    while True:
        buf = from_fd.read(FTP_PAGE_SIZE)
        if len(buf) == 0:
            break

        local_crc = nuttx_crc32(buf, local_crc)
        to_fd.write(buf)
        bar.update(from_fd.tell())

    if not no_verify:
        print_if(verbose, "Verifying...", file=sys.stderr)
        remote_crc = ftp.checksum(file_path)
        if local_crc != remote_crc:
            fault("Verification failed: 0x{local_crc:08x} != 0x{remote_crc:08x}".format(**locals()))
            
    from_fd.close()



    rospy.loginfo('Inited px4logs_manager')

    ftp_list_call = rospy.ServiceProxy('/mavros/ftp/list', FileList)


    mavftp_lock = Lock()
    rospy.spin()


    # AttributeError: __exit__
    # mavftp.do_download_local("/fs/microsd/log/2017-10-20/10_53_02.ulg","logs/10_53_02.ulg",True,True,True)


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
        path_on_rpi = convert_to_rpi_path(_LOGS_DIRECTORY, log['path_on_px4'])
        while not log['downloaded']:
            try:
                with mavftp_lock:
                    time.sleep(1)
                    size_on_px4 = download(log['path_on_px4'], path_on_rpi, accept_operation, True, True, not _CHECK_FILE_CHECKSUM)
                # если успешно донладим, меняем флаг в словаре в памяти
                db.on_download(log, path_on_rpi, size_on_px4)
            except Exception as ex:
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                print("Downloader error: " + str(ex))
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE" & сбросить ftp "rosservice call /mavros/ftp/reset"    
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

def download(file_path, file_name, accept_operation, verbose=True, no_progressbar=False, no_verify=False):
    # optimized transfer size for FTP message payload
    # XXX: bug in ftp.cpp cause a doubling request of last package.
    # -1 fixes that.
    # Размер чанка особо не влияет на скорость порядка 36Kb
    FTP_CHUNK = 239 * 18 - 1

    mavros.set_namespace("/mavros")

    local_crc = 0
    local_file = open(file_name, 'wb')

    print_if(verbose, "Downloading from", file_path, "to", file_name)
    # https://stackoverflow.com/questions/7447284/how-to-troubleshoot-an-attributeerror-exit-in-multiproccesing-in-python

    try:
        #remote_file = ftp.open(file_path, 'r')
        with ftp.open(file_path, 'r') as remote_file:
            if remote_file.size == 0: return 0 #raise Exception("File size = 0!")
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

            return remote_file.size

    except Exception as ex:
        raise Exception("Download error: " + str(ex))
        ftp.reset_server()


def upload(description, additional_feedback, file_path, email, uploader_url, allow_for_analysis = 'false', obfuscated = 'false'):
	
    files = {'filearg': open(file_path,'rb')}
    values = {'description': description, 'feedback': additional_feedback, 'email': email, 'allowForAnalysis': allow_for_analysis, 'obfuscated': obfuscated}
    r = requests.post(uploader_url, files = files, data = values)

    return r.status_code, r.url


def load_param(param, default=None):
    if rospy.has_param(param):
        return rospy.get_param(param)
    elif not default is None:
        print("Param: " + str(param) + " not set & use default value: " + str(default))
        return rospy.get_param(param, default)
    else:
        print("Error: " + str(param) + " not set & have not default value")
        raise SystemExit


def main():
    rospy.init_node('px4logs_manager')

    no_progressbar = False
    try:
        import progressbar as pbar
    except ImportError:
        print("Prigressbar disabled. install python-progressbar", file=sys.stderr)
        no_progressbar = True

    # optimized transfer size for FTP message payload
    # XXX: bug in ftp.cpp cause a doubling request of last package.
    # -1 fixes that.
    FTP_PAGE_SIZE = 239 * 18 - 1


    #####################################################################################
    # Наверное можно это и не использовать
    #####################################################################################
    FTP_PWD_FILE = '/tmp/.mavftp_pwd'


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

#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import rospy
import json
from mavros_msgs.srv import FileList

rospy.init_node('creator_list_px4_ulog')

JSON_PATH='logs/px4logs.json'

# /mavros/ftp/checksum
# /mavros/ftp/close
# /mavros/ftp/list
# /mavros/ftp/mkdir
# /mavros/ftp/open
# /mavros/ftp/read
# /mavros/ftp/remove
# /mavros/ftp/rename
# /mavros/ftp/reset
# /mavros/ftp/rmdir
# /mavros/ftp/truncate
# /mavros/ftp/write

# pi@raspberrypi:~ $ rosservice  
# args  call  find  info  list  type  uri

# флешки, нет в px
# pi@raspberrypi:~ $ rosservice call /mavros/ftp/list /fs/microsd/log
# list: []
# success: True
# r_errno: 0

ftp_list_call = rospy.ServiceProxy('/mavros/ftp/list', FileList)
# rosservice call /mavros/ftp/list /fs/microsd/log

def is_log_in_dict(log_path,logs_dict):
    # ищем такой же лог в словаре
    for _log in logs_dict:
        # в структуре может отсуствовать name
        if (_log['path_on_px4'] == log_path): return True
        # отлов ошибки нужно сделать "AttributeError: 'dict' object has no attribute 'name'"
    return False
    
def load_json(path):
    with open(path, 'r') as infile:
        _dict=json.load(infile)
        # надо поставить оотлов ошибки, если файла нет или в нем невалидный !"[]" json "ValueError: No JSON object could be decoded"
    infile.close()
    return _dict

def dump_json(data,path):
    with open(path, 'w') as outfile:
        json.dump(data, outfile)
    outfile.close

def create_list():
    logs_dict=load_json(JSON_PATH)
    
    # если type равен 0 значит это файл, если 1 то это папка
    # тк пикс не создает супервложенных директорий, можно обойтись без рекурсивного перехода между папками
    main_logs_folder=ftp_list_call('/fs/microsd/log')
    if(main_logs_folder.success and main_logs_folder.r_errno == 0):
        for main_logs_folder_list in main_logs_folder.list:
            if (main_logs_folder_list.type == 1):
                path_current_folder_of_logs="/fs/microsd/log/" + main_logs_folder_list.name
                # проход по папкам с файлами логов
                current_folder_of_logs=ftp_list_call(path_current_folder_of_logs)
                if(current_folder_of_logs.success and current_folder_of_logs.r_errno == 0):
                    for log in current_folder_of_logs.list:
                        if (log.type == 0): # если найден файл лога
                            _log_path = path_current_folder_of_logs + "/" + log.name
                            if not is_log_in_dict(_log_path,logs_dict):
                                logs_dict.append({"path_on_px4":_log_path,"path_on_rpi":"","downloaded":False,"uploaded":False,"url_px4io":""})
                else: print("Import log folder error in " + path_current_folder_of_logs)
    else: print("Import log folder error in /fs/microsd/log")
    dump_json(logs_dict,JSON_PATH)

def main():
    create_list()
    logs_dict=load_json(JSON_PATH)
    print logs_dict

if __name__ == '__main__':
    main()
    