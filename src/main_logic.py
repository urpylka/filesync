
#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

# Copyright 2018-2019 Artem Smirnov

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import os.path
import time
from threading import Thread
from queue import Queue
from fnmatch import fnmatch 

from json_array import JsonArray
from smart_buffer import SmartBuffer
from mover import Mover
from logger import get_logger

def try_again(times, interval, error_message_func, func, *args):
    """
    The function is for trying to execute smthn
    """

    if times < 0:
        raise Exception("Times argument must be positive or zero")
    elif times > 0:
        for iter in range(times):
            try:
                func(*args)
                return
            except Exception as ex:
                error_message_func(ex)
                time.sleep(interval)
        raise Exception("Max iteration exceeding")
    else:
        while True:
            try:
                func(*args)
                return
            except Exception as ex:
                error_message_func(ex)
                time.sleep(interval)


def is_filtered(filename, include=[], exclude=[]):
    """
    Use times = 0 for execute until it will be works!

    The function based on article:
    https://ru.stackoverflow.com/questions/463862/Найти-файлы-по-части-пути-не-только-по-имени-c-python

    Вообще можно фильтровать в теории
    (если решить вопрос с запросом всех данных сразу)
    по след. критериям:
    - min_filesize
    - max_filesize
    - created_interval (start, end)
    - modified_interval (start, end)

    или может прям добавить алгоритм, типа:
    min_filesize = 120 && begin_created = 20190520 || end_modified = 20150811
    """
    for excl in exclude:
        if fnmatch(filename, excl):
            return True
    for incl in include:
        if fnmatch(filename, incl):
            return False
    return True


def finder(number, args):
    """
    Смысла в нескольких finder нет, если только не запросы паралельного листинга,
    что в случае с текущим алгоритмом исключено.

    Или, возможно, это будет полезно для запрашивания инфы
    """

    window, db, source, target, search_interval, mkdir_interval, wq, include, exclude, db_key, db_def_record, logger = args
    logger.debug("Finder-" + str(number) + " was created.")

    while True:
        try:
            dirs = Queue()
            dirs.put('/')

            while True:

                # For waiting if had smthn in db_backup
                # and for waiting worker on cur_dir
                while not wq.empty():
                    time.sleep(1)

                if dirs.empty():
                    break
                else:
                    cur_dir = dirs.get()

                    # Creating directory on target if it doesn't exist
                    try_again(0, mkdir_interval, logger.info, target.mkdir, cur_dir)

                    # Searching for files & dirs
                    for file in source.ls(cur_dir):
                        file = os.path.join(cur_dir, file)
                        #
                        # Сюда можно добавить проверку по маске
                        # (только нужно сделать чтобы маска вроде *.jpg не выбивала папку /dir)
                        #
                        # сейчас имя файла игнорится только со звездочками .DS_Store
                        # а папка например temp, не игнорится
                        #
                        if source.is_dir(file):
                            logger.info("Finder-{0}: Found a new dir: {1}".format(str(number), str(file)))
                            dirs.put(file)
                        else:
                            if is_filtered(file, include, exclude): logger.debug("Filtred: " + str(file))
                            elif not db.in_records(db_key, file):

                                    # Prepare new object
                                    record = db_def_record.copy()
                                    record[db_key] = file

                                    # Save the object
                                    db.append(record)
                                    wq.put(record)
                                    logger.info("Finder-{0}: Found a new file: {1}".format(str(number), str(file)))
                                    window.add_records([record])

                    dirs.task_done()

        except Exception as ex:
            logger.error("Finder-" + str(number) + ": " + str(ex))
        time.sleep(search_interval)


def mover(number, args):

    window, target, source, wq, logger = args

    while True:
        # Object from queue takes by link
        # therefore changes the record gives changes in JsonArray
        element = wq.get()

        # Assigment filesize to db
        # Нет смысла включать это в цикл finder смысла нет,
        # тк все равно везде это делается отдельной операцией
        # element['size'] = source.get_size(element['source_path'])
        def f(elem_p): element['size'] = source.get_size(elem_p)            
        try_again(0, 1, logger.info, f, element['source_path'])
        window.update_record(element)

        m = Mover(logger, source, target, element, number)
        m.move()
        window.update_record(element)

        wq.task_done()