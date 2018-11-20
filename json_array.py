#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import os, json
from threading import Lock

class JsonArray:
    """
    https://m.habr.com/post/186608/
    У меня получается последовательность из словарей, содержащих словари,
    а я хочу получить последовательность словарей
    ИЛИ НЕТ?
    """

    _file_lock = Lock()
    _internal_lock = Lock()
    _records = []

    def __init__(self, json_path, logger):
        self._json_path = json_path
        self._logger = logger
        self._records = self._load_json()
        self._logger.debug("init_db: БД инициализирована")


    def __len__(self):
        with self._internal_lock:
            return len(self._records)


    def __getitem__(self, key):
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

        _records = []
        try:
            with self._file_lock:
                with open(self._json_path, 'r') as infile:
                    _records = json.load(infile)

        except IOError as ex:
            if ex.errno == 2:
                self._logger.debug("load_json: Файл базы данных еще не создан")

        except ValueError as ex:
            self._logger.critical("load_json: Некорректный json: " + str(ex))
            exit(1)

        return _records


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
                with open(self._json_path, 'w', encoding='utf-8') as outfile:
                    json.dump(self._records, outfile)
                    self._logger.debug("dump_json: File of the DB was updated successful!")

    def __del__(self):
            self._dump_json()
            with self._file_lock:
                os.unlink(self._json_path)
