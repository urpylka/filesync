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

from threading import Event

class Target(object):
    """
    Может лучше сделать Grabber or Deleter?

    Как описать общий механизм работы функции, если разные ее исполенения возвращают разный результат?
    Может возвращать ключ-значение? И это ключ-значение писать в DB?

    А еще наверное нужно передавать какие-то блокировки в downloader/uploader
    Например, если передавать большой файл может случится так, что нужно будет притормозить передачу
    """

    is_remote_available = Event()

    def __init__(self, *args):
        raise NotImplementedError()


    def upload(self, local_path, remote_path):
        """
        1. Функция исполняется в вызывающем потоке
        2. Функция должна возвращать True или, если что-то пошло не так, выбрасывать исключение
        3. Если функция возвращает какие-то значения, их нужно передавать по ссылке через аргуемент
        4. Перед тем как запустить функцию нужно проверить событие is_remote_available
        """
        raise NotImplementedError()
