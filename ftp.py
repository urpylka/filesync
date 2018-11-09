#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

from target_uploader import TargetUploader
import time, ftplib
from threading import Thread, Event

class FTP(TargetUploader):

    def __init__(self, conn_params, verbose = False):
        self.verbose = verbose

        self.conn_params = conn_params

        self.is_remote_available = Event()
        self.is_remote_available.clear()

        t = Thread(target = self._connect, args = ())
        t.daemon = True
        t.start()

    def __del__(self):
        self.ftp.quit()

    def _connect(self):
        while True:
            time.sleep(1)
            try:
                self.ftp = ftplib.FTP(self.conn_params[0], self.conn_params[1], self.conn_params[2])
                self.ftp.login()
            except Exception as ex:
                if self.is_remote_available.is_set():
                    self.is_remote_available.clear()
                    print("Раздел недоступен, все операции заблокированы")
                    continue
            if not self.is_remote_available.is_set():
                self.is_remote_available.set()
                print("Раздел доступен, все операции разблокированы")

    def upload(self, local_path, remote_path):
        try:
            with open(local_path, 'rb') as fobj:
                self.ftp.storbinary('STOR ' + remote_path, fobj, 1024)
            # with open(path) as fobj:
            #     ftp.storlines('STOR ' + path, fobj)
        except Exception as ex:
            print "Error on upload to FTP server: " + str(ex)
            return False
        return True
