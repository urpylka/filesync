#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:


import os, json
from threading import Lock
from Queue import Queue

class JSONDatabase:
    def __init__(self, json_path):
        self._json_path = json_path
        self._logs_records = self._load_json(json_path)
        self._init_work()
        print("Инициализирована БД")


    def _init_work(self):
        self.dq = Queue()
        self.uq = Queue()
        for _log in self._logs_records:
            if not _log['downloaded']: self.dq.put(_log)
            elif not _log['uploaded'] and _log['size_on_px4'] > 0: self.uq.put(_log)


    def on_download(self, _log, path_on_rpi, size_on_px4):
        """
        Функция успешного завершения загрузки лога на RPI.

        По-хорошему надо все эти операции сделать атомарными.
        Также нет смысла блокировать доступ к _log,
        тк обращение к этим полям в другом месте в принципе нет

        dump_json исполняется после uq.put тк это более долгая операция,
        теоретически способная заблокировать следующую операцию uploader
        """
        _log["downloaded"] = True
        _log["path_on_rpi"] = path_on_rpi
        _log["size_on_px4"] = size_on_px4
        if size_on_px4 > 0: self.uq.put(_log)
        else: print("File size = 0! " + path_on_rpi)
        self._dump_json(self._logs_records, self._json_path)
        print("Downloaded " + _log["path_on_px4"] + " to " + path_on_rpi)
        self.dq.task_done()


    def on_upload(self, _log, url_px4io):
        _log["uploaded"] = True
        _log["url_px4io"] = url_px4io
        self._dump_json(self._logs_records, self._json_path)
        print("Uploaded " + _log["path_on_rpi"] + " to " + url_px4io)
        self.uq.task_done()


    def is_log_in_records(self, log_path):
        """
        Функция поиска лога лога в словаре.
    
        В структуре словаря может отсутствовать атрибут name,
        и нужно сделать отлов ошибки
        AttributeError: 'dict' object has no attribute 'name'
        """
        for _log in self._logs_records:
            if _log['path_on_px4'] == log_path: return True
        return False


    def on_find(self, path_on_px4):
        """
        Функция добавления в массив найденного нового лога.

        dump_json исполняется после dq.put тк это более долгая операция,
        теоретически способная заблокировать следующую операцию downloader
        """
        print("Find " + path_on_px4)
        self._logs_records.append({"path_on_px4":path_on_px4, "size_on_px4":"", "path_on_rpi":"", "downloaded":False, "uploaded":False, "url_px4io":""})
        self.dq.put(self._logs_records[len(self._logs_records) - 1])
        self._dump_json(self._logs_records, self._json_path)


    _file_lock = Lock()


    def _load_json(self, path):
        """
        Функция для загрузки словаря из json-файла.
        """
        try:
            with self._file_lock:
                    with open(path, 'r') as infile:
                            records = json.load(infile)

        except ValueError as ex:
            print("Ошибка: некорректный json: " + str(ex))

        except IOError as ex:
            if ex.errno == 2:
                records = []
                print("Файл базы данных не создан, да и фиг с ним: " + str(ex))

        return records


    def _dump_json(self, data, path):
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
