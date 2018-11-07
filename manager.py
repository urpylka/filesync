#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import os.path, time
from json_database import JSONDatabase
from flir_duo import FlirDuoCamera
from threading import Thread, Event

def convert_to_rpi_path(local_directory, remote_path):
    return local_directory + '/' + os.path.basename(os.path.dirname(remote_path)).replace('-','') + '_' + os.path.basename(remote_path).replace('_','')

def downloader(i, db, files, local_directory):
    print("Created the Downloader-" + str(i))
    """
    Function for downloading files from remote device
    """
    while True:
        file = db.dq.get()
        local_path = convert_to_rpi_path(local_directory, file['remote_path'])
        while not file['downloaded']:
            try:
                files.is_remote_available.wait()
                if files.download(file['remote_path'], local_path):
                    db.on_download(file, local_path)
            except Exception as ex:
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                print("Downloader error: " + str(ex))
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE" & сбросить ftp "rosservice call /mavros/ftp/reset"    
                time.sleep(2)
                # вообще можно перейти к другому элементу из очереди

def create_downloaders(count, db, files, local_directory):
    for i in range(count):
      t = Thread(target = downloader, args = (i, db, files, local_directory, ))
      t.daemon = True
      t.start()

def finder(db, files, search_interval, verbose = True):
    print("Created the Finder")
    """
    Function for searching files on remote device
    """
    while True:

        files.is_remote_available.wait()

        try:
            print("Searching a new files...")
            my_list = files.get_list_of_files()
            if verbose: print(my_list)
            for item in my_list:
                if not db.is_file_in_records(item): db.on_find(item)

        except Exception as ex:
            print("Finder error: " + str(ex))

        time.sleep(search_interval)

def create_finder(db, files, search_interval):
    t = Thread(target = finder, args = (db, files, search_interval, ))
    t.daemon = True
    t.start()

# def load_param(param, default=None):
#     if rospy.has_param(param):
#         return rospy.get_param(param)
#     elif not default is None:
#         print("Param: " + str(param) + " not set & use default value: " + str(default))
#         return rospy.get_param(param, default)
#     else:
#         print("Error: " + str(param) + " not set & have not default value")
#         raise SystemExit

def main():

    # SEARCH_INTERVAL = load_param('~search_interval', 10)
    # DOWNLOADERS_COUNT = load_param('~downloaders_count', 5)
    # LOCAL_DIRECTORY = load_param('~local_directory')
    # REMOTE_DIRECTORY = load_param('~remote_directory')
    # DB_JSON_PATH = load_param('~db_json_path')

    SEARCH_INTERVAL = 10
    DOWNLOADERS_COUNT = 5
    LOCAL_DIRECTORY = "/home/pi/flir"
    REMOTE_DIRECTORY = "/"
    DB_JSON_PATH = "/home/pi/flir.json"
    UUID = "66F8-E5D9"
    FILES_EXTENTIONS = ['jpg', 'png']

    db = JSONDatabase(DB_JSON_PATH)
    files = FlirDuoCamera(UUID, FILES_EXTENTIONS)

    create_downloaders(DOWNLOADERS_COUNT, db, files, LOCAL_DIRECTORY)
    create_finder(db, files, SEARCH_INTERVAL)
    #db.dq.join()
    while True:
        time.sleep(10)

if __name__ == '__main__':
    main()
