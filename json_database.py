#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import os, json
from threading import Lock
from Queue import Queue

class JSONDatabase:
    def __init__(self, json_path):
        self._json_path = json_path
        self._files_records = self._load_json(json_path)
        self._init_work()
        print("Инициализирована БД")


    def _init_work(self):
        self.dq = Queue()
        #self.uq = Queue()
        for _file in self._files_records:
            if not _file['downloaded']: self.dq.put(_file)
            #elif not _file['uploaded'] and _file['size_on_flash'] > 0: self.uq.put(_file)


    def on_download(self, file, local_path):
        """
        Функция успешного завершения загрузки лога на RPI.

        По-хорошему надо все эти операции сделать атомарными.
        Также нет смысла блокировать доступ к _file,
        тк обращение к этим полям в другом месте в принципе нет

        dump_json исполняется после uq.put тк это более долгая операция,
        теоретически способная заблокировать следующую операцию uploader
        """
        file["downloaded"] = True
        file["local_path"] = local_path
        self._dump_json(self._files_records, self._json_path)
        print("Downloaded " + file["source_path"] + " to " + local_path)
        self.dq.task_done()


    def on_upload(self, _file, url_px4io):
        _file["uploaded"] = True
        _file["url_px4io"] = url_px4io
        self._dump_json(self._files_records, self._json_path)
        print("Uploaded " + _file["local_path"] + " to " + url_px4io)
        self.uq.task_done()


    def is_file_in_records(self, log_path):
        """
        Функция поиска лога лога в словаре.
    
        В структуре словаря может отсутствовать атрибут name,
        и нужно сделать отлов ошибки
        AttributeError: 'dict' object has no attribute 'name'
        """
        for _file in self._files_records:
            if _file['source_path'] == log_path: return True
        return False


    def on_find(self, source_path):
        """
        Функция добавления в массив найденного нового лога.

        dump_json исполняется после dq.put тк это более долгая операция,
        теоретически способная заблокировать следующую операцию downloader
        """
        print("Find " + str(source_path))
        self._files_records.append({"source_path":source_path, "local_path":"", "downloaded":False})
        self.dq.put(self._files_records[len(self._files_records) - 1])
        self._dump_json(self._files_records, self._json_path)


    _file_lock = Lock()


    def _load_json(self, path):
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
