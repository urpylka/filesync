#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

class SourceDownloader(object):

    is_remote_available = Event()
    db_fileds = []
    verbose = False

    def __init__(self, *args):
        raise NotImplementedError()

    def get_list_of_files(self):
        raise NotImplementedError()
    
    def download(self, remote_path, local_path):
        raise NotImplementedError()

    def del_source_file(self, remote_path, local_path):
        raise NotImplementedError()
