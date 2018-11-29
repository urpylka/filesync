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

from device_abstract import Device

import time, ftplib, socket, logging
from threading import Thread

class FTP(Device):
    """
    target = FTP("192.168.0.10", "test-1", "passwd", logging)
    target.upload("/home/pi/flir/20181113_205519_20181113212352517.JPG", "20181113_205519_20181113212352517.JPG")
    """

    def __init__(self, *args):
        self.host, self.user, self.passwd, self._logger = args

        t = Thread(target = self._connect, args = ())
        t.daemon = True
        t.start()


    def __del__(self):
        self._ftp.abort()
        self._ftp.close()


    def _connect(self):
        self.is_remote_available.clear()
        self._logger.info("TARGET: FTP недоступен, все операции заблокированы")

        while True:
            time.sleep(1)
            try:
                self._ftp = ftplib.FTP(self.host)
                self._ftp.login(self.user, self.passwd)
                if not self.is_remote_available.is_set():
                    self.is_remote_available.set()
                    self._logger.info("TARGET: FTP доступен, все операции разблокированы")
                    break
            except Exception as ex:
                # ошибка 111 - если хост недоступен
                self._logger.debug("TARGET: " + str(ex))
                if self.is_remote_available.is_set():
                    self.is_remote_available.clear()
                    self._logger


    def upload(self, local_path, remote_path):
        try:
            # with open(local_path, 'rb') as fobj:
            #     res = self._ftp.storbinary('STOR ' + remote_path, fobj, 1024)

            self.is_remote_available.wait()
            self._ftp.cwd('/')
            res = self._ftp.storbinary('STOR ' + '/' + remote_path, open(local_path, 'rb'))
            if not res.startswith('226 Transfer complete'):
                raise Exception("File was not uploaded successful: " + res)
        except Exception as ex:
            raise ex
        return True
