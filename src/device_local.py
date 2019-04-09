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

import os
from device_abstract import Device
from bash_commands import *

class LOCAL(Device):
    """
    LOCAL(root_dir="/")
    """

    def _connect(self):
        """
        Не нужно выполнять никаких действий

        Мб только создать папку, если ее не существует
        """
        pass


    def get_list(self):
        """
        Get list of files
        """
        self.is_remote_available.wait()
        my_list = []
        for rootdir, dirs, files in os.walk(self.kwargs["root_path"]):
            for file in files:
                my_list.append(os.path.join(rootdir.replace(self.kwargs["root_path"], '', 1), file))
        return my_list


    def download(self, device_path, target_stream):
        """
        1. Функция исполняется в вызывающем потоке
        2. Функция должна возвращать True или, если что-то пошло не так, выбрасывать исключение
        3. Если функция возвращает какие-то значения, их нужно передавать по ссылке через аргуемент
        """
        target_stream.ram_to_flash(device_path)
        #???? типа надо как-то подтвердить что файл сохранен


    def upload(self, source_stream, device_path):
        """
        1. Функция исполняется в вызывающем потоке
        2. Функция должна возвращать True или, если что-то пошло не так, выбрасывать исключение
        3. Если функция возвращает какие-то значения, их нужно передавать по ссылке через аргуемент
        """
        source_stream.ram_to_flash(device_path)
        #???? типа надо как-то подтвердить что файл сохранен


    def delete(self, remote_path):
        """
        1. Функция исполняется в вызывающем потоке
        2. Функция должна возвращать True или, если что-то пошло не так, выбрасывать исключение
        3. Если функция возвращает какие-то значения, их нужно передавать по ссылке через аргуемент
        """
        delete(remote_path)
