#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

# https://github.com/stilliard/docker-pure-ftpd
# https://github.com/stilliard/docker-pure-ftpd/wiki/Basic-example-walk-through

from target_abstract import Target
import time, ftplib, socket
from threading import Thread, Event

class FTP(Target):

    is_remote_available = Event()

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
            #     print("urpylka")
            res = self._ftp.storbinary('STOR ' + '/' + remote_path, open(local_path, 'rb'))
            # res = self._ftp.storbinary('STOR ' + remote_path, fobj, 1024)
            if not res.startswith('226 Transfer complete'):
                return False
        except Exception as ex:
            print("urpylka exception")
            raise ex
        return True
