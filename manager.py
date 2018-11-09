#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:


from json_database import JSONDatabase
from flir_duo import FlirDuoCamera, download
from ftp import FTP

import os.path, time
from threading import Thread, Event


def downloader(number, args):
    db, source, local_directory = args
    verbose = True
    print("Created the Downloader-" + str(number))
    """
    Function for downloading files from remote device
    """
    while True:
        file = db.dq.get()
        source_path = file['source_path']
        local_path = local_directory + '/' + os.path.basename(os.path.dirname(source_path)).replace('-','') + '_' + os.path.basename(source_path).replace('_','')
        if verbose: print(local_path)
        if verbose: print(source_path)
        while not file['downloaded']:
            try:
                source.is_remote_available.wait()
                if download(source_path, local_path):
                    db.on_download(file, local_path)
            except Exception as ex:
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                print("Downloader-" + str(number) + "error: " + str(ex))
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE" & сбросить ftp "rosservice call /mavros/ftp/reset"    
                time.sleep(2)
                # вообще можно перейти к другому элементу из очереди


def finder(number, args):
    db, source, search_interval = args
    verbose = True
    print("Created the Finder-" + str(number))
    """
    Function for searching files on remote device
    """
    while True:
        source.is_remote_available.wait()
        try:
            print("Searching a new source...")
            my_list = source.get_list_of_files()
            if my_list != None:
                if verbose: print("List of source: " + str(my_list))
                for item in my_list:
                    if not db.is_file_in_records(item): db.on_find(item)
            elif verbose: print("List of source is None")

        except Exception as ex:
            print("Finder-" + str(number) + "error: " + str(ex))

        time.sleep(search_interval)


def uploader(number, args):
    db, target = args
    print("Created the Uploader-" + str(number))
    while True:

        file = db.uq.get()
        local_path = file['local_path']
        target_path = '/' + os.path.basename(local_path)

        while not file['uploaded']:
            target.is_remote_available.wait()
            try:
                if target.upload(local_path, target_path):
                    db.on_upload(file, target_path)
            except Exception as ex:
                print("Uploader-" + str(number) + "error: " + str(ex))
                time.sleep(2)


def create_threads(count, function, *args):
    for i in range(count):
        t = Thread(target = function, args = (i, args,))
        t.daemon = True
        t.start()


def main():

    db = JSONDatabase("/home/pi/flir/db.json")
    source = FlirDuoCamera("66F8-E5D9", ['JPG', 'png'])
    target = FTP(["localhost","",""])

    create_threads(1, finder, db, source, 10)
    create_threads(5, downloader, db, source, "/home/pi/flir")
    create_threads(3, uploader, db, target)

    while True:
        time.sleep(10)

if __name__ == '__main__':
    main()
