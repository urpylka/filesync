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
        # local_path = local_directory + '/' + os.path.basename(os.path.dirname(source_path)).replace('-', '') + '_' + os.path.basename(source_path).replace('_', '')
        local_path = local_directory + '/' + os.path.basename(source_path)

        # local_path = record['local_path']
        target_path = '/' + os.path.basename(local_path)
        temp_target_path = target_path + ".part"

        logger.debug("Worker-" + str(number) + ": source_path " + source_path + " local_path " + local_path)

        buffer_stream = SmartBuffer(record['source_size'], logger, 0, None, os.path.basename(local_path) + ".temp")
        # если задать размер буффера меньше,
        # перезапустить программу с большим буффером (в файл),
        # то не знаю будет ли перезаписываться или какие-то еще глюки

        iter = 0
        while not record['downloaded'] or not record['uploaded'] or not record['dropped']:
            
            iter += 1

            # не делать wq.done()
            # if iter == 10:
            #     break

            try:
                logger.info("Worker-" + str(number) + ": " + str(source_path) + " starting worker. Iteration: " + str(iter))

                if not record["uploaded"]:
                    u = in_thread(target.upload, buffer_stream, temp_target_path, 400000)        # сосёт

                    if not record["downloaded"] or not buffer_stream.is_wrote_all():
                    # может вообще убрать эту проверку
                        d = in_thread(source.download, source_path, buffer_stream, 1000000) # вставляет

                        d.join()
                        logger.debug("Worker-" + str(number) + ": downloader")
                        if buffer_stream.is_wrote_all():
                            record["downloaded"] = True
                            # record["local_path"] = local_path
                            logger.info("Worker-" + str(number) + ": " + str(source_path) + " was downloaded")

                    u.join()
                    logger.debug("Worker-" + str(number) + ": uploader")
                    if buffer_stream.is_read_all():
                        record["uploaded"] = True
                        logger.info("Worker-" + str(number) + ": " + str(source_path) + " was uploaded")


                if record["downloaded"] and record["uploaded"]:
                    target.rename(temp_target_path, target_path)
                    source.delete(record["source_path"])
                    record["dropped"] = True
                    logger.info("Worker-" + str(number) + ": " + str(source_path) + " was deleted")

            except Exception as ex:
                logger.error("Worker-" + str(number) + ": " + str(ex) + " with file " + source_path)
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE"
                # & сбросить ftp "rosservice call /mavros/ftp/reset"    
                # вообще, в случае этой ошибки можно перейти к другому элементу из очереди
                time.sleep(2)

        wq.task_done()
        del(buffer_stream)
        logger.info("Worker-" + str(number) + ": File " + source_path + " was downloaded to " + target_path)


def in_thread(function, *args):
    #name='Worker-1'
    t = Thread(target=function, args=(args[0], args[1], args[2],))
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

                logger.info("Downloader-" + str(number) + ": local_path " + local_path + " size: " + str(size))

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

    config_path = "./config.json"

    config = {}
    if os.path.exists(os.path.dirname(config_path)):
        try:
            import json
            with open(config_path, 'r') as infile:
                config = json.load(infile)

        except IOError as ex:
            if ex.errno == 2:
                print("Error 1. Config file doesn't exixst")
                exit(1)

        except ValueError as ex:
            print("Error 2. Incorrect Json in config file: " + str(ex))
            exit(1)
    else:
        print("Error 3. Config file doesn't exixst")
        exit(1)


    if len(config["workers"]) < 1:
        print("Error 4. Count of worker less then 1")
        exit(1)
    else:
        for worker_data in config["workers"]:
            if not worker_data["disable"]:
                logger = get_logger(worker_data["name"], worker_data["logger"]["log_path"], worker_data["logger"]["log_level"])

                db = JsonArray(worker_data["db"]["db_path"], worker_data["db"]["autosave_interval"], logger)
                wq = Queue()
                for record in db:
                    if not record['downloaded'] or not record['uploaded'] or not record['dropped']: wq.put(record)

                m1 = __import__(worker_data["source"]["module_path"])
                worker_data["source"]["args"]["logger"] = logger
                source = getattr(m1, worker_data["source"]["device_class"])(**worker_data["source"]["args"])

                m2 = __import__(worker_data["target"]["module_path"])
                worker_data["target"]["args"]["logger"] = logger
                target = getattr(m2, worker_data["target"]["device_class"])(**worker_data["target"]["args"])

                create_threads(worker_data["finder"]["count"], finder, db, source, worker_data["finder"]["finder_interval"], wq, worker_data["finder"]["extensions"], logger)
                create_threads(worker_data["count"], worker, target, source, wq, logger)

    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        # del db
        # smart_buffer.dump()
        return 0


if __name__ == '__main__':
    main()

# rm -f flir/db.json && clear && sudo ./src/manager.py
# while :; do sleep 1; clear; ls -l Sherlock.s03e01.avi; done
