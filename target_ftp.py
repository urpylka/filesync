#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

# https://github.com/stilliard/docker-pure-ftpd
# https://github.com/stilliard/docker-pure-ftpd/wiki/Basic-example-walk-through

from target_abstract import Target
import time, ftplib
from threading import Thread, Event

class CustomFTP(ftplib.FTP):
    """
    https://stackoverflow.com/questions/50060037/ftplib-connectionrefusederror-errno-111-connection-refused-python-3-5
    """
    def makepasv(self):
        if self.af == socket.AF_INET:
            host, port = ftplib.parse227(self.sendcmd('PASV'))
        else:
            host, port = ftplib.parse229(self.sendcmd('EPSV'), self.sock.getpeername())

        if '0.0.0.0' == host:
            """ this ip will be unroutable, we copy Filezilla and return the host instead """
            host = self.host
        return host, port

class FTP(Target):

    is_remote_available = Event()

    def __init__(self, *args):
        self.host, self.user, self.passwd, self._logger = args

        t = Thread(target = self._connect, args = ())
        t.daemon = True
        t.start()


    def __del__(self):
        self.ftp.abort()
        self.ftp.close()


    def _connect(self):
        self.is_remote_available.clear()
        self._logger.debug("TARGET: FTP недоступен, все операции заблокированы")

        while True:
            time.sleep(1)
            try:
                self.ftp = CustomFTP(self.host)
                self.ftp.login(self.user, self.passwd)
                if not self.is_remote_available.is_set():
                    self.is_remote_available.set()
                    self._logger.debug("TARGET: FTP доступен, все операции разблокированы")
                    break
            except Exception as ex:
                self._logger.error("TARGET: " + str(ex))
                if self.is_remote_available.is_set():
                    self.is_remote_available.clear()
                    self._logger


    def upload(self, local_path, remote_path):
        try:
            with open(local_path, 'rb') as fobj:
                res = self.ftp.storbinary('STOR ' + remote_path, fobj, 1024)
                if not res.startswith('226 Transfer complete'):
                    return False
            # with open(path) as fobj:
            #     ftp.storlines('STOR ' + path, fobj)
        except Exception as ex:
            self._logger.error("TARGET: Error on upload to FTP server: " + str(ex) + " on file " + local_path)
            return False
        return True
