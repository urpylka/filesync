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

import io
from threading import Lock

class SmartBuffer(object):
    """
    Smart buffer w fixed size
    (worked on cyclic quenue)
    async (w lock between read & write,
    w different cursor in read & write)
    """

    threads_lock = Lock()

    def __init__(self, buffer_size, buffer_type=0):
        """
        Buffer can be placed in memory & flash

        если указать размер буффера равный размеру файла,
        тип буффера - файл
        и имя файла куда мы хотим его сохранить,
        то получится local
        """

        self.size = buffer_size

        if buffer_type == 0:
            self.buffer = io.BytesIO() #size
        elif buffer_type == 1:
            self.buffer = io.open()

        self.pos_r = 0
        self.pos_w = 0

    
    def read(self, chunk_size):
        """
        Функция не гарантирует возвращения всего чанка,
        даже если не достигнут конец файла

        Можно сделать счетчик сколько мы читаем из файла,
        заведя переменную available + self.already_read
        """
        with self.threads_lock:

            diff = self.pos_w - self.pos_r

            # если pos_w >= pos_r
            # AND chunk_size > 0,
            # что, логично, то проверка
            # diff >= 0 не нужна
            if diff >= chunk_size:
                # читаем целый чанк
                self.buffer.seek(self.pos_r)
                self.pos_r += chunk_size
                return self.buffer.read(chunk_size)
            else:
                # читаем сколько есть
                # если доступно меньше чанка
                if diff > 0:
                    # читаем до pos_w
                    self.buffer.seek(self.pos_r)
                    self.pos_r = self.pos_w
                    #self.pos_r += diff
                    return self.buffer.read(diff)
                elif diff < 0:
                    # читаем, что осталось
                    available = self.size - self.pos_r

                    self.buffer.seek(self.pos_r)
                    self.pos_r = 0
                    return self.buffer.read(available)
                else:
                    # diff == 0
                    # блокировать, если передача не закончилась
                    return b''


    def read2(self, chunk_size):
        with self.threads_lock:

            diff = self.pos_w - self.pos_r

            if diff > 0:
                #available = chunk_size if diff >= chunk_size else diff
                available = min(chunk_size, diff)

                self.buffer.seek(self.pos_r)
                self.pos_r += available
                return self.buffer.read(available)
            elif diff < 0:
                # здесь скорее всего ошибка получается,
                # тк если даунлодер убежал вперед
                # это не значит что осталось меньше чанка до конца

                # читаем, что осталось
                available = self.size - self.pos_r

                self.buffer.seek(self.pos_r)
                self.pos_r = 0
                return self.buffer.read(available)
            else:
                # diff == 0
                # блокировать, если передача не закончилась
                return b''


    def write(self, chunk):
        """
        """
        with self.threads_lock:
            pass

    def __del__(self):
        """
        """

        self.buffer.close()
