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


class my_ftp(ftplib.FTP):
    def storbinary(self, cmd, fp, blocksize=8192, callback=None, rest=None):
        """
        Пришлось исправить стандартный метод
        перестановкой вызова callback
        """
        self.voidcmd('TYPE I')
        with self.transfercmd(cmd, rest) as conn:
            while 1:
                buf = fp.read(blocksize)
                if not buf: break

                if callback: callback(buf)
                conn.sendall(buf)

        return self.voidresp()


class FTP(Device):
    """
    target = FTP("192.168.0.10", "test-1", "passwd", logging)
    with open("/home/pi/flir/20181113_205519_20181113212352517.JPG", 'rb') as source_stream:
        target.upload(source_stream, "20181113_205519_20181113212352517.JPG")

    Можно сделать ввод количества параллельных соединений и сделать вместо блокировки семафор

    Два раза пишет что FTP недоступен
    """

    _internal_lock = RLock()
    _ftp = ftplib.FTP()

    def __del__(self):
        self._ftp.abort()
        self._ftp.close()


    def _connect(self):
        self.is_remote_available.clear()
        self.kwargs["logger"].info("TARGET: FTP is unavailble. All operations is lock")

        while True:
            time.sleep(1)

            # starttime = time.time()
            retry = False
            try:
                self._ftp.voidcmd("NOOP")
            except:
                retry = True

            while retry:
                try:
                    self._ftp.connect(self.kwargs["host"])
                    self._ftp.login(self.kwargs["user"], self.kwargs["passwd"])
                    retry = False

                    if not self.is_remote_available.is_set():
                        self.is_remote_available.set()
                        self.kwargs["logger"].info("TARGET: FTP is availble. All operations is unlock")

                except IOError as ex:
                    retry = True
                    # self.kwargs["logger"].info("TARGET: Time disconnected - " + str(time.time() - starttime))

                    # ошибка 111 - если хост недоступен
                    self.kwargs["logger"].debug("TARGET: " + str(ex))
                    if self.is_remote_available.is_set():
                        self.is_remote_available.clear()
                        self.kwargs["logger"].info("TARGET: FTP is unavailble. All operations is lock")


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
                        self.kwargs["logger"].info("TARGET: File was not uploaded to server yet: " + exc)
                        return 0
                    else:
                        raise Exception("TARGET: Can't get file size on ftp server: " + exc)


    def upload(self, source_stream, device_path, chunk_size=8192, offset=0):

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

        self.kwargs["logger"].info("TARGET: Uploading: " + str(device_path))

        with self._internal_lock:
            self.is_remote_available.wait()

            # без этого будет работать?
            self._ftp.cwd(os.path.dirname(device_path))

            res = None
            while 1:
                time.sleep(1)
                self.kwargs["logger"].debug("TARGET: enter upload")

                try:
                    already_sent = self.get_size(device_path) #  already upload wo errors
                    self.kwargs["logger"].info("TARGET: Started w " + str(already_sent))
                    source_stream.seek(already_sent)

                    self.is_remote_available.wait()
                    res = self._ftp.storbinary("STOR " + device_path, source_stream, blocksize=chunk_size, rest=already_sent)

                    if not res.startswith("200 I successfully done nothin"):
                        if not res.startswith("226 Transfer complete"):
                            raise Exception("File was not uploaded successful: " + res)
                    
                    break
                except Exception as ex:

                    # Можно реализовать метод seek для reader’a
                    # С исключением если воайтер уже догнал ридер
                    # и сик не может исполнится или еще чего

                    # Обычно когда возникает ошибка здесь,
                    # мне кажется нужно заново выполнять
                    # transfercmd("STOR " + device_path, self.rest)

                    # тк ошибок может быть очень много и разных,
                    # и не всегда известно записалась часть файла
                    # или нет и лучше начать с той позиции в которой ошибок не было

                    # эта штука умеет как минимум выбрасывать
                    # [Errno 32] Broken pipe
                    # [Errno 104] Connection reset by peer

                    # При тестировании, при прерывании сразу
                    # было пустое сообщение об ошибке
                    # будто throw Exception()

                    self.kwargs["logger"].error("TARGET: " + str(ex))
            self.kwargs["logger"].debug("TARGET: exit upload")
