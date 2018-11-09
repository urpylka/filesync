#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import os, json
from threading import Lock
from Queue import Queue

class FilesRecords:

    _file_lock = Lock()

    def __init__(self, json_path):
        self._json_path = json_path
        self.files_records = self.load_json()

        self.dq = Queue()
        self.uq = Queue()
        for _file in self.files_records:
            if not _file['downloaded']: self.dq.put(_file)
            elif not _file['uploaded']: self.uq.put(_file)

        print("Инициализирована БД")


    def in_records(self, source_path):
        """
        Функция поиска лога лога в словаре.
    
        В структуре словаря может отсутствовать атрибут name,
        и нужно сделать отлов ошибки
        AttributeError: 'dict' object has no attribute 'name'
        """
        for _file in self.files_records:
            if _file['source_path'] == source_path: return True
        return False


    def load_json(self, path = self._json_path):
        """
        Функция для загрузки словаря из json-файла.
        """
        records = []
        try:
            with self._file_lock:
                with open(path, 'r') as infile:
                    records = json.load(infile)

        except ValueError as ex:
            print("Ошибка: некорректный json: " + str(ex))

        except IOError as ex:
            if ex.errno == 2:
                #records = []
                print("Файл базы данных не создан, да и фиг с ним: " + str(ex))

        return records

    def dump_json(self, data = self.files_records, path = self._json_path):
        """
        Функция для сохранения словаря в json-файл.
        """
        with self._file_lock:

            if not os.path.exists(os.path.dirname(path)):
                try:
                    os.makedirs(os.path.dirname(path))
                except OSError as exc:  # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise

            with open(path, 'w') as outfile:
                json.dump(data, outfile)
