#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

class TargetUploader(object):

    is_remote_available = Event()
    db_fileds = []
    verbose = False

    def __init__(self, conn_params, verbose = False):
        raise NotImplementedError()

    def upload(self, local_path, remote_path):
        """
        Must return bool
        """
        raise NotImplementedError()

    # Может лучше сделать Grabber or Deleter
    # def del_source_file(self, remote_path, local_path, verbose = True):
    #     pass
