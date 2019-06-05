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

import os, json, time
from threading import Thread, Lock

class JsonArray:
    """
    More info about sequence there:
    https://m.habr.com/post/186608/

    Examples:

    _record = { "source_path": "AAAA", "downloaded": False, "local_path": "", "uploaded": False, "target_path": "" }
    ja.append(_record)

    _record2 = { "source_path": "BBBB", "downloaded": True, "local_path": "", "uploaded": False, "target_path": "" }
    ja.append(_record2)

    print(ja.in_records("source_path","AAA"))
    print(ja.in_records("source_path","AAAA"))
    print(ja.in_records("source_path","ABBB"))
    print(ja.in_records("source_path","BBBB"))
    """

    _file_lock = Lock()
    _internal_lock = Lock()
    _records = []

    def __init__(self, json_path, autosaver_delay, logger):
        self._json_path = json_path
        self._logger = logger
        self._records = self._load_json()

        t = Thread(target = self._autosaver, args = (autosaver_delay,))
        t.daemon = True
        t.start()

        self._logger.debug("Initializated DB successful")


    def _autosaver(self, delay):
        while(True):
            self._dump_json()
            time.sleep(delay)


    def __len__(self):
        with self._internal_lock:
            return len(self._records)


    def __getitem__(self, key):
        # хотя одновременно читать элемент в принципе можно, только если при этом не пишем
        # если значение или тип ключа некорректны, list выбросит исключение
        with self._internal_lock:
            return self._records[key]


    def __setitem__(self, key, value):
        with self._internal_lock:
            self._records[key] = value


    def __delitem__(self, key):
        with self._internal_lock:
            del self._records[key]


    def __iter__(self):
        with self._internal_lock:
            return iter(self._records)


    def __reversed__(self):
        with self._internal_lock:
            return FunctionalList(reversed(self._records))


    def append(self, value):
        with self._internal_lock:
            self._records.append(value)


    def in_records(self, key, value):
        """
        Функция поиска лога лога в словаре.
    
        В структуре словаря может отсутствовать атрибут name,
        и нужно сделать отлов ошибки
        AttributeError: 'dict' object has no attribute 'name'
        """
        with self._internal_lock:
            for record in self._records:
                if record[key] == value: return True
            return False


    def _load_json(self):
        """
        Функция для загрузки словаря из json-файла.
        """

        records = []
        try:
            with self._file_lock:
                with open(self._json_path, 'r') as infile:
                    records = json.load(infile)

        except IOError as ex:
            if ex.errno == 2:
                self._logger.debug("load_json: File of DB was created")

        except ValueError as ex:
            self._logger.error("load_json: Incorrect Json in file of DB: " + str(ex))

        return records


    def _dump_json(self):
        """
        Функция для сохранения словаря в json-файл.
        """

        with self._file_lock:

            if not os.path.exists(os.path.dirname(self._json_path)):
                try:
                    os.makedirs(os.path.dirname(self._json_path))
                except OSError as exc:  # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise

            with self._internal_lock:
                with open(self._json_path, 'w') as outfile:
                    json.dump(self._records, outfile)
                    # self._logger.debug("dump_json: File of the DB was updated successful!")


    def __del__(self):
            self._dump_json()
            # для удаления файла при удалении объекта
            # with self._file_lock:
            #     os.unlink(self._json_path)
