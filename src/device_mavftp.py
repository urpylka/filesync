#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

# Copyright 2018 Artem Smirnov

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from device_abstract import Device

import time, sys, os
from threading import Thread, Lock

import rospy, mavros
# from mavros.utils import *
from mavros_msgs.srv import FileList

class ProgressBar:
    """
    Wrapper class for hiding file transfer progressbar construction
    """

    def __init__(self, quiet, operation, maxval):
        no_progressbar = False
        try:
            import progressbar as pbar
        except ImportError:
            print("Prigressbar disabled. install python-progressbar", file=sys.stderr)
            no_progressbar = True

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


# def _resolve_path(path = None):
#     """
#     Resolve FTP path using PWD file
#     """
#     FTP_PWD_FILE = '/tmp/.mavftp_pwd'
#     if os.path.exists(FTP_PWD_FILE):
#         with open(FTP_PWD_FILE, 'r') as fd:
#             pwd = fd.readline()
#     else:
#         # default home location is root directory
#         pwd = os.environ.get('MAVFTP_HOME', '/')

#     if not path:
#         return os.path.normpath(pwd)    # no path - PWD location
#     elif path.startswith('/'):
#         return os.path.normpath(path)   # absolute path
#     else:
#         return os.path.normpath(os.path.join(pwd, path))


class MAVFTP(Device):

    ftp_list = rospy.ServiceProxy('/mavros/ftp/list', FileList)

    def __init__(self, *args):
        self.logdir = args
        self.mavftp_lock = Lock()

        rospy.loginfo('Inited px4logs_manager')
        rospy.init_node("mavftp", anonymous=True)
        # mavros.set_namespace("/mavros")

        t = Thread(target = self._px4_available, args = ())
        t.daemon = True
        t.start()


    def _px4_available(self):
        rospy.spin()


    def download(self, remote_path, local_path):
    """
    кароче качать можно и нужно вообще побайтно (в принципе как я понял можно уже в полете),
    но туповато это самому реализовывать поэтому возьму это из mavftp
    """

    with mavftp_lock:

        verbose = True
        verify = True

        # optimized transfer size for FTP message payload
        # XXX: bug in ftp.cpp cause a doubling request of last package.
        # -1 fixes that.
        FTP_CHUNK = 239 * 18 - 1
        # Change of chunk size doesn't most affect to download speed ~36Kb

        mavros.set_namespace("/mavros")

        local_crc = 0
        local_file = open(file_name, 'wb')

        print_if(verbose, "Downloading from", file_path, "to", file_name)
        # https://stackoverflow.com/questions/7447284/how-to-troubleshoot-an-attributeerror-exit-in-multiproccesing-in-python

        try:
            # /opt/ros/kinetic/lib/python2.7/dist-packages/mavros/ftp.py
            with mavros.ftp.open(file_path, 'r') as remote_file:

                if remote_file.size == 0:
                    raise Exception("File size = 0!")

                bar = ProgressBar(no_progressbar, "Downloading: ", remote_file.size)

                while True:

                    buf = remote_file.read(FTP_CHUNK)
                    if len(buf) == 0:
                        break

                    local_crc = mavros.nuttx_crc32(buf, local_crc)
                    local_file.write(buf)
                    bar.update(remote_file.tell())

                    if verify:
                        print_if(verbose, "Verifying...")
                        remote_crc = mavros.ftp.checksum(file_path)
                        if local_crc != remote_crc:
                            raise Exception("Verification failed: 0x{local_crc:08x} != 0x{remote_crc:08x}".format(**locals()))

        # Posible errors:

        # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
        # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE" & сбросить ftp "rosservice call /mavros/ftp/reset"    

        # AttributeError: __exit__
        # mavftp.do_download_local("/fs/microsd/log/2017-10-20/10_53_02.ulg","logs/10_53_02.ulg",True,True,True)

        # донладим (по хорошему нужна проверка что лог уже не скачен и имя не повторяется)
        # rosrun mavros mavftp download /fs/microsd/log/2017-10-20/13_29_53.ulg 13_29_53.ulg
        
        # pi@raspberrypi:~/urpylka_px4logs $ rosservice call /mavros/ftp/open /fs/microsd/log/2017-10-20/10_53_02.ulg 0
        # size: 1676016
        # success: True
        # r_errno: 0
        
        # pi@raspberrypi:~/urpylka_px4logs $ rosservice call /mavros/ftp/read /fs/microsd/log/2017-10-20/10_53_02.ulg 0 1676015
        # data: []
        # success: False
        # r_errno: 5
        
        # ошибка 9 возникает когда файл не открыт
        
        # pi@raspberrypi:~/urpylka_px4logs $ rosservice call /mavros/ftp/read /fs/microsd/log/2017-10-20/10_53_02.ulg 0 100
        # data: [85, 76, 111, 103, 1, 18, 53, 1, 58, 201, 155, 24, 0, 0, 0, 0, 40, 0, 66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 0, 73, 15, 99, 104, 97, 114, 91, 52, 48, 93, 32, 118, 101, 114, 95, 115, 119, 53, 56, 50, 48, 98, 102, 98, 55, 57, 52, 48, 101, 99, 54, 57, 53, 55, 51, 99, 97, 101, 51]
        # success: True
        # r_errno: 0


        except Exception as ex:
            mavros.ftp.reset_server()
            raise Exception("Download error: " + str(ex))
        finally:
            local_file.close()

        return remote_file.size


def get_list(self):
    root_path = "/fs/microsd/log"
    array = []
    while True:
        try:
            with mavftp_lock:

                logs_folder = self.ftp_list(root_path)
                if not logs_folder.success or logs_folder.r_errno != 0:
                    raise Exception("Import log folder error in " + root_path)
                else:
                    # проход по корневой папке /fs/microsd/log
                    for logs_folder_list in logs_folder.list:

                        # если type равен 0, значит это файл, если 1, то это папка
                        # тк пикс не создает супервложенных директорий для логов,
                        # можно обойтись без рекурсивного перехода между папками
                        if logs_folder_list.type == 1:

                            path_session_logs_folder = root_path + '/' + logs_folder_list.name

                            # проход по папкам с файлами логов
                            session_logs_folder = self.ftp_list(path_session_logs_folder)

                            if not session_logs_folder.success or session_logs_folder.r_errno != 0:
                                raise Exception("Import log folder error in " + path_session_logs_folder)
                            else:
                                for log in session_logs_folder.list:

                                    # если найден файл лога
                                    if log.type == 0:
                                        array.append(path_session_logs_folder + "/" + log.name)

        except Exception as ex:
            raise Exception("Finder error: " + str(ex))
        return array

def main():

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

    # Так происходит когда в px нет флешки
    # pi@raspberrypi:~ $ rosservice call /mavros/ftp/list /fs/microsd/log
    # list: []
    # success: True
    # r_errno: 0


if __name__ == '__main__':
    main()
