#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import os, json
from threading import Lock

class FilesRecords:

    _file_lock = Lock()
    files_records = []

    def __init__(self, json_path, logging):
        self._json_path = json_path
        self._logging = logging
        self.files_records = self.load_json()
        self._logging.debug("init_db: БД инициализирована")


    def in_records(self, key, value):
        """
        Функция поиска лога лога в словаре.
    
        В структуре словаря может отсутствовать атрибут name,
        и нужно сделать отлов ошибки
        AttributeError: 'dict' object has no attribute 'name'
        """
        for _file in self.files_records:
            if _file[key] == value: return True
        return False


    def load_json(self):
        """
        Функция для загрузки словаря из json-файла.
        """
        path = self._json_path

        records = []
        try:
            with self._file_lock:
                with open(path, 'r') as infile:
                    records = json.load(infile)

        except IOError as ex:
            if ex.errno == 2:
                self._logging.debug("load_json: Файл базы данных еще не создан")

        except ValueError as ex:
            self._logging.critical("load_json: Некорректный json: " + str(ex))
            exit

        return records


    def dump_json(self):
        """
        Функция для сохранения словаря в json-файл.
        """
        path = self._json_path

        with self._file_lock:

            if not os.path.exists(os.path.dirname(path)):
                try:
                    os.makedirs(os.path.dirname(path))
                except OSError as exc:  # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise

            with open(path, 'w') as outfile:
                json.dump(self.files_records, outfile)
                self._logging.debug("dump_json: File of the DB was updated successful!")
