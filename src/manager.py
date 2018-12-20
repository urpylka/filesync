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

import os.path
import time
from threading import Thread
from queue import Queue

from json_array import JsonArray
from device_disk import DISK
from device_ftp import FTP
from logger import get_logger
from smart_buffer import SmartBuffer

def finder(number, args):

    db, source, search_interval, dq, files_extensions, logger = args
    logger.debug("Finder-" + str(number) + " was created.")

    while True:
        try:
            for item in source.get_list():

                # Check extension
                if files_extensions.count(item["path"].split('.')[-1]) == 1:

                    if not db.in_records("source_path", item["path"]):
                        logger.info("Finder-{0}: Found a new file: {1}".format(str(number), str(item["path"])))

                        # prepare the new object
                        record = {}.copy()
                        record["source_path"] = item["path"]
                        record["source_size"] = item["size"]
                        record["source_hash"] = item["hash"]
                        record["downloaded"] = False
                        record["dropped"] = False
                        record["uploaded"] = False
                        record["target_path"] = ""

                        record["local_path"] = ""

                        # save the new object
                        db.append(record)
                        dq.put(record)

        except Exception as ex:
            logger.error("Finder-" + str(number) + ": " + str(ex))

        time.sleep(search_interval)


def worker(number, args):

    target, source, wq, logger = args
    logger.debug("Worker-" + str(number) + " was created.")

    local_directory = ""

    while True:
        # объект из очереди передается по ссылке,
        # поэтому изменение record приведет к изменению record в JsonArray
        record = wq.get()

        source_path = record['source_path']
        local_path = local_directory + '/' + os.path.basename(os.path.dirname(source_path)).replace('-', '') + '_' + os.path.basename(source_path).replace('_', '')

        # local_path = record['local_path']
        target_path = '/' + os.path.basename(local_path)

        logger.debug("Worker-" + str(number) + ": source_path " + source_path + " local_path " + local_path)

        buffer_stream = SmartBuffer(record['source_size'])

        while not record['downloaded'] and not record['uploaded'] and not record['dropped']:

            try:

                # чтобы при срыве чего-либо все продолжалось с того же места,
                # нужно чтобы буффер не очищался,
                # а source.download и target.upload не теряли указатели

                d = in_thread(source.download, source_path, buffer_stream) # вставляет
                u = in_thread(target.upload, buffer_stream, target_path)   # сосёт

                d.join()
                if buffer_stream.already_wrote:
                    record["downloaded"] = True
                    # record["local_path"] = local_path

                u.join()
                if buffer_stream.already_read:
                    record["uploaded"] = True

                if record["downloaded"] and record["uploaded"]:
                    source.delete(record["source_path"])
                    record["dropped"] = True

            except Exception as ex:
                logger.error("Worker-" + str(number) + ": " + str(ex) + " with file " + source_path)
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE"
                # & сбросить ftp "rosservice call /mavros/ftp/reset"    
                # вообще, в случае этой ошибки можно перейти к другому элементу из очереди
                time.sleep(2)

        wq.task_done()
        logger.info("Worker-" + str(number) + ": File " + source_path + " was downloaded to " + target_path)


def in_thread(function, *args):
    #name='Worker-1'
    t = Thread(target=function, args=(args,))
    t.daemon = True
    t.start()
    return t


def downloader(number, args):

    source, local_directory, dq, uq, logger = args
    logger.debug("Downloader-" + str(number) + " was created.")

    while True:
        # объект из очереди передается по ссылке,
        # поэтому изменение record приведет к изменению record в JsonArray
        record = dq.get()
        source_path = record['source_path']
        # local_path = local_directory + '/' + os.path.basename(os.path.dirname(source_path)).replace('-', '') + '_' + os.path.basename(source_path).replace('_', '')
        local_path = local_directory + '/' + os.path.basename(source_path)
        logger.info("Downloader-" + str(number) + ": source_path " + source_path + " local_path " + local_path)
        while not record['downloaded']:
            try:
                #if source.download(source_path, local_path):
                size = 0
                try:
                    # иногда бывает ошибка с возвращением
                    # к позиции связанной с размером файла
                    size = os.stat(local_path).st_size
                except:
                    pass
                with open(local_path, 'wb+') as target_stream:
                    target_stream.seek(size)
                    source.download(source_path, target_stream)

                record["downloaded"] = True
                record["local_path"] = local_path
                dq.task_done()
                logger.info("Downloader-" + str(number) + ": File " + source_path + " was downloaded to " + local_path)
                uq.put(record)
            except Exception as ex:
                logger.error("Downloader-" + str(number) + ": " + str(ex) + " with file " + source_path)
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE" & сбросить ftp "rosservice call /mavros/ftp/reset"    
                # вообще, в случае этой ошибки можно перейти к другому элементу из очереди
                time.sleep(2)


def uploader(number, args):
    """
    Очень интересная ошибка будет, если в target не будет метода upload
    (пустая, тк берется из абстрактного класса)
    """
    target, uq, logger = args
    logger.debug("Uploader-" + str(number) + " was created.")
    while True:
        record = uq.get()
        local_path = record['local_path']
        target_path = '/' + os.path.basename(local_path)

        logger.info("Uploader-" + str(number) + ": source_path " + record['source_path'] + " local_path " + local_path)

        while not record['uploaded']:
            try:
                #if target.upload(local_path, target_path):
                with open(local_path, 'rb') as source_stream:
                    target.upload(source_stream, target_path)

                record['uploaded'] = True
                record['target_path'] = target_path
                uq.task_done()
                logger.info("Uploader-" + str(number) + ": File " + local_path + " was uploaded to " + target_path)
            except Exception as ex:
                logger.error("Uploader-{0}: {1}: with file {2} and target path {3}".format(str(number), str(ex), local_path, target_path))
                time.sleep(2)


def create_threads(count, function, *args):
    for i in range(count):
        t = Thread(target=function, args=(i+1, args,))
        t.daemon = True
        t.start()


def main():
    logger = get_logger("filesync", "/home/pi/filesync/flir/filesync.log", "INFO")
    source = DISK(uuid="1234-5678", mount_point="/mnt", logger=logger)
    # source = DISK(uuid="66F8-E5D9", mount_point="/mnt", logger=logger)
    target = FTP(host="192.168.0.10", user="test-1", passwd="passwd", logger=logger)
    db = JsonArray("/home/pi/filesync/flir/db.json", 5, logger)

    dq = Queue()
    uq = Queue()
    for record in db:
        if not record['downloaded']: dq.put(record)
        elif not record['uploaded']: uq.put(record)

    create_threads(1, finder, db, source, 10, dq, ["JPG", "jpg", "MOV", "mov", "TIFF", "tiff", "avi", "AVI"], logger)
    create_threads(5, downloader, source, "/home/pi/filesync/flir", dq, uq, logger)
    create_threads(1, uploader, target, uq, logger)

    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        del db
        # smart_buffer.dump()
        return 0

if __name__ == '__main__':
    main()
