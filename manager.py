#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

from json_database import FilesRecords
from source_flir_duo import FlirDuoCamera
from target_ftp import FTP

import os.path, time
from threading import Thread, Event
from Queue import Queue
import logging


def downloader(number, args):
    db, source, local_directory, dq, uq, logging = args
    logging.debug("Downloader-" + str(number) + " was created.")
    """
    Function for downloading files from remote device
    """
    while True:
        # объект из очереди передается по ссылке,
        # поэтому изменение file приведет к изменению record в db
        file = dq.get()
        source_path = file['source_path']
        local_path = local_directory + '/' + os.path.basename(os.path.dirname(source_path)).replace('-','') + '_' + os.path.basename(source_path).replace('_','')
        logging.debug("Downloader-" + str(number) + ": local_path " + local_path)
        logging.debug("Downloader-" + str(number) + ": source_path " + source_path)
        while not file['downloaded']:
            try:
                source.is_remote_available.wait()
                if source.download(source_path, local_path):
                    file["downloaded"] = True
                    file["local_path"] = local_path
                    # объект _files_records уже изменен,
                    # тк объект file = dq.get() был передан по ссылке
                    db.dump_json()
                    dq.task_done()
                    logging.info("Downloader-" + str(number) + ": File " + source_path + " was downloaded to " + local_path)
                    uq.put(file)
            except Exception as ex:
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                logging.error("Downloader-" + str(number) + ": " + str(ex) + " with file " + source_path)
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE" & сбросить ftp "rosservice call /mavros/ftp/reset"    
                time.sleep(2)
                # вообще, в случае этой ошибки можно перейти к другому элементу из очереди


def finder(number, args):
    db, source, search_interval, record, key, dq, logging = args
    logging.debug("Finder-" + str(number) + " was created.")
    """
    Function for searching files on remote device
    """
    while True:
        source.is_remote_available.wait()
        try:
            logging.debug("Finder-" + str(number) + ": Searching a new files in source...")
            my_list = source.get_list_of_files()
            if my_list != None:
                logging.debug("Finder-" + str(number) + ": List of source: " + str(my_list))
                for item in my_list:
                    if not db.in_records(key, item):
                        logging.info("Finder-" + str(number) + ": Found a new file: " + str(item))
                        # prepare the new object
                        record[key] = item
                        # save the new object
                        db.files_records.append(record)
                        db.dump_json()
                        # add the new object to the upload queue
                        dq.put(db.files_records[len(db.files_records) - 1])
            else: logging.debug("Finder-" + str(number) + ": List of source is None")

        except Exception as ex:
            logging.error("Finder-" + str(number) + ": " + str(ex))

        time.sleep(search_interval)


def uploader(number, args):
    db, target, uq, logging = args
    verbose = True
    logging.debug("Uploader-" + str(number) + " was created.")
    while True:

        file = uq.get()
        local_path = file['local_path']
        target_path = '/' + os.path.basename(local_path)

        while not file['uploaded']:
            target.is_remote_available.wait()
            try:
                if target.upload(local_path, target_path):
                    file['uploaded'] = True
                    file['target_path'] = target_path
                    db.dump_json()
                    uq.task_done()
                    logging.info("Uploader-" + str(number) + ": File " + local_path + " was uploaded to " + target_path)
            except Exception as ex:
                logging.error("Uploader-" + str(number) + ": " + str(ex) + " with file " + local_path)
                time.sleep(2)


def create_threads(count, function, *args):
    for i in range(count):
        t = Thread(target = function, args = (i+1, args,))
        t.daemon = True
        t.start()


def main():

    # Logger
    # https://python-scripts.com/logging-python
    logger = logging.getLogger("filesync")
    logger.setLevel(logging.INFO)
    # create the logging file handler
    fh = logging.FileHandler("/home/pi/flir/filesync.log")
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    fh.setFormatter(formatter)
    # add handler to logger object
    logger.addHandler(fh)

    db = FilesRecords("/home/pi/flir/db.json", logging)

    dq = Queue()
    uq = Queue()
    for _record in db.files_records:
        if not _record['downloaded']: dq.put(_record)
        elif not _record['uploaded']: uq.put(_record)

    source = FlirDuoCamera("66F8-E5D9", ['JPG', 'png'], "/mnt", logging)
    target = FTP("192.168.0.10", "test-1", "passwd", logging)

    record = { "source_path": "", "downloaded": False, "local_path": "", "uploaded": False, "target_path": "" }
    key = "source_path"

    create_threads(1, finder, db, source, 10, record, key, dq, logging)
    create_threads(5, downloader, db, source, "/home/pi/flir", dq, uq, logging)
    create_threads(3, uploader, db, target, uq, logging)

    while True:
        time.sleep(10)

if __name__ == '__main__':
    main()
