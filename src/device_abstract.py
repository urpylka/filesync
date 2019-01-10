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

from threading import Thread
from threading import Event

class Device(object):
    """
    1. Как описать общий механизм работы функции,
    если разные ее исполенения возвращают разный результат?
    Может возвращать ключ-значение? И это ключ-значение писать в DB?
    Или функция должна возвращать True/False?

    2. А еще наверное нужно передавать какие-то блокировки в downloader/uploader
    Если передавать большой файл может случится так, что нужно будет притормозить передачу
    Например, Arming коптера

    3. Может сделать фукнции donwload и upload с использованием потоков,
    таким образом подставив file.open() можно будет писать в файл,
    а если их направить друг на друга они будут писать без сохранения в local

    4. Library for continue a interrupting downloads

    5. Загрузка чанками requests
    https://stackoverflow.com/questions/13909900/progress-of-python-requests-post

    6. Надо добавить в вывод "Not implemented method"
    наименование класса в котором это все вызывается
    """

    def __init__(self, **kwargs):
        self.kwargs = kwargs

        self.is_remote_available = Event()

        t = Thread(target=self._connect, args=())
        t.daemon = True
        t.start()


    def _connect(self):
        raise NotImplementedError("Not implemented method")


    def get_list(self):
        """
        1. Функция исполняется в вызывающем потоке
        2. Функция должна возвращать список файлов (пустой список, если файлов нет)
        или, если что-то пошло не так, выбрасывать исключение

        3. Функция должна возвращать список словарей {"path":"", "size":"", "hash":""}
        """
        raise NotImplementedError("Not implemented method")


    def download(self, device_path, target_stream, chunk_size=1024, offset=0):
        """
        Пока не знаю
        1. Функция исполняется в вызывающем потоке
        2. Функция должна возвращать True или, если что-то пошло не так, выбрасывать исключение
        3. Если функция возвращает какие-то значения, их нужно передавать по ссылке через аргуемент
        """
        raise NotImplementedError("Not implemented method")


    def upload(self, source_stream, device_path, chunk_size=1024, offset=0):
        """
        Пока не знаю
        1. Функция исполняется в вызывающем потоке
        2. Функция должна возвращать True или, если что-то пошло не так, выбрасывать исключение
        3. Если функция возвращает какие-то значения, их нужно передавать по ссылке через аргуемент
        """
        raise NotImplementedError("Not implemented method")


    def delete(self, remote_path):
        """
        1. Функция исполняется в вызывающем потоке
        2. Функция должна возвращать True или, если что-то пошло не так, выбрасывать исключение
        3. Если функция возвращает какие-то значения, их нужно передавать по ссылке через аргуемент
        """
        raise NotImplementedError("Not implemented method")
