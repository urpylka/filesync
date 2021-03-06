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

import os, time, csv
from threading import Thread, Lock

class CsvList:
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

    def __init__(self, csv_path, autosaver_delay, logger):
        self._csv_path = csv_path
        self._logger = logger
        self._records = self._load_csv()
        self._fieldnames = []
        self._data = ["first_name,last_name,city".split(","),
            "Dedric,Medhurst,Stiedemannberg".split(",")
            ]

        t = Thread(target = self._autosaver, args = (autosaver_delay,))
        t.daemon = True
        t.start()

        self._logger.debug("Initializated DB successful")


    def _autosaver(self, delay):
        while(True):
            self._dump_csv()
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
        Функция поиска лога в словаре.
    
        В структуре словаря может отсутствовать атрибут name,
        и нужно сделать отлов ошибки
        AttributeError: 'dict' object has no attribute 'name'
        """
        with self._internal_lock:
            for record in self._records:
                if record[key] == value: return True
            return False


    def _load_csv(self):
        """
        Функция для загрузки словаря из csv-файла.
        """

        records = []
        try:
            with self._file_lock:
                with open(self._csv_path, 'r') as infile:
                    records = csv.DictReader(infile, delimiter=',')

        except IOError as ex:
            if ex.errno == 2:
                self._logger.debug("load_csv: File of DB was created")

        except ValueError as ex:
            self._logger.critical("load_csv: Incorrect Json in file of DB: " + str(ex))
            exit(1)

        return records


    def _dump_csv(self):
        """
        Функция для сохранения словаря в csv-файл.
        """

        with self._file_lock:

            if not os.path.exists(os.path.dirname(self._csv_path)):
                try:
                    os.makedirs(os.path.dirname(self._csv_path))
                except OSError as exc:  # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise

            with self._internal_lock:
                with open(self._csv_path, 'w', newline='') as outfile:
                    writer = csv.DictWriter(outfile, delimiter=',', fieldnames=self._fieldnames)
                    writer.writeheader()

                    self._records

                    for row in data:
                        writer.writerow(row)

                    self._logger.debug("dump_csv: File of the DB was updated successful!")


    def __del__(self):
            self._dump_csv()
            # для удаления файла при удалении объекта
            # with self._file_lock:
            #     os.unlink(self._csv_path)
