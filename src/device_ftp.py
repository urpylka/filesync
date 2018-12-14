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
from threading import Lock

from device_abstract import Device

class FTP(Device):
    """
    target = FTP("192.168.0.10", "test-1", "passwd", logging)
    with open("/home/pi/flir/20181113_205519_20181113212352517.JPG", 'rb') as source_stream:
        target.upload(source_stream, "20181113_205519_20181113212352517.JPG")

    Можно сделать ввод количества параллельных соединений и сделать вместо блокировки семафор

    Два раза пишет что FTP недоступен
    """

    _internal_lock = Lock()
    _ftp = ftplib.FTP()

    def __del__(self):
        self._ftp.abort()
        self._ftp.close()


    def _connect(self):
        self.is_remote_available.clear()
        self.kwargs["logger"].info("TARGET: FTP недоступен, все операции заблокированы")

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
                        self.kwargs["logger"].info("TARGET: FTP доступен, все операции разблокированы")

                except IOError as ex:
                    retry = True
                    # self.kwargs["logger"].info("TARGET: Time disconnected - " + str(time.time() - starttime))

                    # ошибка 111 - если хост недоступен
                    self.kwargs["logger"].debug("TARGET: " + str(ex))
                    if self.is_remote_available.is_set():
                        self.is_remote_available.clear()
                        self.kwargs["logger"].info("TARGET: FTP недоступен, все операции заблокированы")


    def upload(self, source_stream, device_path, chunk_size=1024):
        self.is_remote_available.wait()

        with self._internal_lock:
            self._ftp.cwd(os.path.dirname(device_path))
            res = self._ftp.storbinary('STOR ' + device_path, source_stream, callback=callback1)

            if not res.startswith('226 Transfer complete'):
                raise Exception("File was not uploaded successful: " + res)

def callback1():
    print("urpylka")
