#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:
#
# Copyright 2017 Smirnov Artem.
#

from __future__ import print_function

import os
import rospy
import mavros
from mavros.utils import *
from mavros.nuttx_crc32 import *

# /opt/ros/kinetic/lib/python2.7/dist-packages/mavros/ftp.py
from mavros import ftp

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
#####################################################################################


class ProgressBar:
    """
    Wrapper class for hiding file transfer brogressbar construction
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


# AttributeError: __exit__
# mavftp.do_download_local("/fs/microsd/log/2017-10-20/10_53_02.ulg","logs/10_53_02.ulg",True,True,True)
def do_download_local(file_path,file_name,verbose=True,no_progressbar=False,no_verify=False):
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
