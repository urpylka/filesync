#! /usr/bin/env python3
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

# https://github.com/stilliard/docker-pure-ftpd
# https://github.com/stilliard/docker-pure-ftpd/wiki/Basic-example-walk-through

import os
import time
import ftplib
from threading import RLock

from device_abstract import Device


# class my_ftp(ftplib.FTP):

#     # try:
#     #     import ssl
#     # except ImportError:
#     #     _SSLSocket = None
#     # else:
#     #     _SSLSocket = ssl.SSLSocket

#     def my_storbinary(self, cmd, fp, blocksize=8192, callback=None, rest=None):
#         """
#         Пришлось исправить стандартный метод
#         перестановкой вызова callback
#         """
#         self.voidcmd('TYPE I')
#         with self.transfercmd(cmd, rest) as conn:
#             while 1:
#                 buf = fp.read(blocksize)
#                 if not buf: break

#                 if callback: callback(buf)
#                 conn.sendall(buf)

#         return self.voidresp()


#     def storbinary(self, cmd, fp, blocksize=8192, callback=None, rest=None):
#         """Store a file in binary mode.  A new port is created for you.

#         Args:
#           cmd: A STOR command.
#           fp: A file-like object with a read(num_bytes) method.
#           blocksize: The maximum data size to read from fp and send over
#                      the connection at once.  [default: 8192]
#           callback: An optional single parameter callable that is called on
#                     each block of data after it is sent.  [default: None]
#           rest: Passed to transfercmd().  [default: None]

#         Returns:
#           The response code.
#         """
#         self.voidcmd('TYPE I')
#         with self.transfercmd(cmd, rest) as conn:
#             while 1:
#                 buf = fp.read(blocksize)
#                 if not buf:
#                     break
#                 conn.sendall(buf)
#                 if callback:
#                     callback(buf)
#             # shutdown ssl layer
#             if _SSLSocket is not None and isinstance(conn, _SSLSocket):
#                 conn.unwrap()
#         return self.voidresp()


#     def retrbinary(self, cmd, callback, blocksize=8192, rest=None):
#             """Retrieve data in binary mode.  A new port is created for you.

#             Args:
#             cmd: A RETR command.
#             callback: A single parameter callable to be called on each
#                         block of data read.
#             blocksize: The maximum number of bytes to read from the
#                         socket at one time.  [default: 8192]
#             rest: Passed to transfercmd().  [default: None]

#             Returns:
#             The response code.
#             """
#             self.voidcmd('TYPE I')
#             with self.transfercmd(cmd, rest) as conn:
#                 while 1:
#                     data = conn.recv(blocksize)
#                     if not data:
#                         break
#                     callback(data)
#                 # shutdown ssl layer
#                 if _SSLSocket is not None and isinstance(conn, _SSLSocket):
#                     conn.unwrap()
#             return self.voidresp()


class FTP(Device):
    """
    target = FTP("192.168.0.10", "test-1", "passwd", logging)
    with open("/home/pi/flir/20181113_205519_20181113212352517.JPG", 'rb') as source_stream:
        target.upload(source_stream, "20181113_205519_20181113212352517.JPG")

    Можно сделать ввод количества параллельных соединений и сделать вместо блокировки семафор

    Два раза пишет что FTP недоступен

    TODO:
    * Параллельная работа с FTP (отдельные коннекшены)
    * Проверка hash-суммы
    """

    _internal_lock = RLock()
    _ftp = ftplib.FTP()

    def __del__(self):
        self._ftp.abort()
        self._ftp.close()


    def _connect(self):
        self.is_remote_available.clear()
        self._prefix = self.kwargs["user"] + "@" + self.kwargs["host"] + ": "

        self.kwargs["logger"].info(self._prefix + "FTP is unavailble. All operations is lock")

        while True:
            time.sleep(1)

            # starttime = time.time()
            retry = False
            try: self._ftp.voidcmd("NOOP")
            except: retry = True

            while retry:
                try:
                    self._ftp.connect(self.kwargs["host"])
                    self._ftp.login(self.kwargs["user"], self.kwargs["passwd"])
                    retry = False

                    if not self.is_remote_available.is_set():
                        self.is_remote_available.set()
                        self.kwargs["logger"].info(self._prefix + "FTP is availble. All operations is unlock")

                except ftplib.error_perm as ex_perm:
                    # retry = True

                    self.kwargs["logger"].error(self._prefix + "_connect(): " + str(ex_perm))
                    if self.is_remote_available.is_set():
                        self.is_remote_available.clear()
                        self.kwargs["logger"].info(self._prefix + "FTP is unavailble. All operations is lock")

                except IOError as ex:
                    # retry = True
                    # self.kwargs["logger"].info("TARGET: Time disconnected - " + str(time.time() - starttime))

                    # ошибка 111 - если хост недоступен
                    self.kwargs["logger"].debug(self._prefix + "_connect(): " + str(ex))
                    if self.is_remote_available.is_set():
                        self.is_remote_available.clear()
                        self.kwargs["logger"].info(self._prefix + "FTP is unavailble. All operations is lock")


    def get_size(self, device_path):

        while 1:
            with self._internal_lock:
                self.is_remote_available.wait()

                try:
                    self.is_remote_available.wait()
                    self._ftp.voidcmd('TYPE I')
                    response = self._ftp.size(device_path)

                    if not response is None:
                        return response

                except Exception as ex:
                    # если файла еще нет, нужно продолжить с длиной в ноль
                    exc = str(ex)
                    if exc.startswith("550"):
                        self.kwargs["logger"].debug(self._prefix + "File was not uploaded to server yet: " + exc)
                        return 0
                    else:
                        raise Exception(self._prefix + "Can't get file size on ftp server: " + exc)


    def upload(self, source_stream, device_path, chunk_size=8192):

        # f_blocksize = 1024
        # total_size = os.path.getsize(file_path)
        # size_written = 0

        # # http://qaru.site/questions/15601924/ftplib-storbinary-with-ftps-is-hangingnever-completing
        # def handle(block):
        #     global size_written
        #     global total_size
        #     global f_blocksize
        #     size_written = size_written + f_blocksize if size_written + f_blocksize < total_size else total_size
        #     percent_complete = size_written / total_size * 100
        #     print("%s percent complete" %str(percent_complete))

        # def _cb(self, buf):
        #     """
        #     Метод в первую очередь для ведения статистики количества
        #     записанный чанков в FTP

        #     Также можно считать количество информации записанной для ведения стастики
        #     """
        #     self.rest += len(self.buf)
        #     self.buf = buf

        with self._internal_lock:
            self.is_remote_available.wait()
            self.kwargs["logger"].debug(self._prefix + "Uploading " + str(device_path))
            while 1:
                self.is_remote_available.wait()
                try:
                    # без этого будет работать?
                    self._ftp.cwd(os.path.dirname(device_path))

                    already_sent = self.get_size(device_path) #  already upload wo errors
                    self.kwargs["logger"].info(self._prefix + "Uploading " + str(device_path) + " Started w " + str(already_sent))
                    source_stream.seek(already_sent)

                    self.is_remote_available.wait()
                    # res = self._ftp.storbinary("STOR " + device_path, source_stream, blocksize=chunk_size, rest=already_sent)

                    self._ftp.voidcmd('TYPE I')
                    with self._ftp.transfercmd("STOR " + device_path, already_sent) as conn:
                        while 1:
                            # source_stream.show_stat()
                            buf = source_stream.read(chunk_size)
                            if not buf:
                                break
                            conn.sendall(buf)
                            # shutdown ssl layer
                            # if _SSLSocket is not None and isinstance(conn, _SSLSocket):
                            # conn.unwrap()
                            self.kwargs["logger"].debug(self._prefix + "Wrote to server: " + str(len(buf)))
                            # source_stream.show_stat()

                    res = self._ftp.voidresp()

                    if not res.startswith("200 I successfully done nothin"):
                        if not res.startswith("226 Transfer complete"):
                            raise Exception("File was not uploaded successful: " + res)

                    self.kwargs["logger"].debug(self._prefix + "End of the uploading")
                    break

                except Exception as ex:

                    # [Errno 32] Broken pipe
                    # [Errno 104] Connection reset by peer
                    # Пустой Exception()

                    self.kwargs["logger"].error(self._prefix + "Uploading was interrupted: " + str(ex))
                    time.sleep(1)


    def download(self, device_path, target_stream, chunk_size=8192):

        with self._internal_lock:
            self.is_remote_available.wait()
            self.kwargs["logger"].debug(self._prefix + "Downloading " + str(device_path))
            while 1:
                self.is_remote_available.wait()
                try:
                    # без этого будет работать?
                    self._ftp.cwd(os.path.dirname(device_path))

                    already_sent = target_stream.tell() #  already upload wo errors
                    self.kwargs["logger"].info(self._prefix + "Resume w " + str(already_sent))

                    res = self._ftp.retrbinary("RETR " + device_path, target_stream.write, blocksize=chunk_size, rest=already_sent)
                    if not res.startswith("200 I successfully done nothin"):
                        if not res.startswith("226 Transfer complete"):
                            raise Exception("File was not uploaded successful: " + res)

                    break

                except Exception as ex:
                    self.kwargs["logger"].error(self._prefix + "Downloading was interrupted: " + str(ex))
                    time.sleep(1)


    def rename(self, old_path, new_path):

        with self._internal_lock:
            self.is_remote_available.wait()
            self.kwargs["logger"].info(self._prefix + "Renaming " + str(old_path) + " to "+ str(new_path))
            while 1:
                self.is_remote_available.wait()
                try:
                    # без этого будет работать?
                    self._ftp.cwd(os.path.dirname(old_path))

                    self._ftp.rename(old_path, new_path)

                    break

                except Exception as ex:
                    self.kwargs["logger"].error(self._prefix + "Renaming was interrupted: " + str(ex))
                    time.sleep(1)


    def get_list(self):
        """
        Get list of files
        """

        rootdir = '/'
        with self._internal_lock:
            self.is_remote_available.wait()

            # без этого будет работать?
            self._ftp.cwd(os.path.dirname(rootdir))

            my_list = []

            for filename in self._ftp.nlst():

                path = os.path.join(rootdir, filename)
                size = self.get_size(path)

                my_list.append({"path": path, "size": size, "hash": ""})
            return my_list


    def delete(self, device_path):

        with self._internal_lock:
            self.is_remote_available.wait()
            self.kwargs["logger"].info(self._prefix + "Deleting " + str(device_path))
            while 1:
                self.is_remote_available.wait()
                try:
                    # без этого будет работать?
                    self._ftp.cwd(os.path.dirname(device_path))

                    self._ftp.delete(device_path)

                    break

                except Exception as ex:
                    self.kwargs["logger"].error(self._prefix + "Deleting was interrupted: " + str(ex))
                    time.sleep(1)
