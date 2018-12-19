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

    https://docs.python.org/3/library/io.html

    Если долго не идет процесс read кешировать все в локал, если не файл не большой не более 1Гб
    или какие-то лимиты

    Можно сделать чтобы буффер сам подзатирался, скачивался,
    а если не скачивался, то становился больше
    """

    threads_lock = Lock()

    def __init__(self, file_size, buf_size=None, buf_type=0):
        """
        Buffer can be placed in memory or flash

        если указать размер буффера равный размеру файла,
        тип буффера - файл
        и имя файла куда мы хотим его сохранить,
        то получится local
        """

        self.file_size = file_size
        self.buf_type = buf_type

        if buf_size is None:
            self.buf_size = self.get_optimize_buf_size()
        else:
            self.buf_size = buf_size

        if buf_type == 0:
            self.buffer = io.BytesIO()
        elif buf_type == 1:
            self.buffer = io.open("file.temp", "wb")

        self.pos_r = 0
        self.pos_w = 0

        self.already_read = 0
        self.already_wrote = 0

        # self.is_error = False
        # self.last_read = b''

    def _read(self, chunk_size):
        """
        Функция умеет читать только до границы буффера
        """
        self.buffer.seek(self.pos_r)
        buf = self.buffer.read(chunk_size)

        self.pos_r += chunk_size

        if self.pos_r == self.buf_size:
            self.pos_r = 0

        self.already_read += chunk_size
        return buf


    def read(self, chunk_size):
        if chunk_size < 0:
            # В аналогичных функциях read chunk_size
            # может равняться -1 тогда будет весь буффер
            raise Exception("Размер чанка не может быть отрицательным")

        with self.threads_lock:
            diff = self.pos_w - self.pos_r

            # если pos_w >= pos_r
            # AND chunk_size > 0,
            # что, логично, то проверка
            # diff >= 0 не нужна
            if diff >= chunk_size:
                # читаем целый чанк
                return self._read(chunk_size)
            else:
                # если доступно меньше чанка
                needs = 0
                buf = b''

                if diff > 0:
                    # читаем до pos_w
                    buf = self._read(diff)
                    needs = chunk_size - diff

                elif diff < 0:
                    # читаем, что осталось на этой стороне
                    available = self.buf_size - self.pos_r
                    needs = chunk_size - available
                    buf = self._read(available)

                else:
                    # diff == 0
                    needs = chunk_size

                # нужно чтобы в первый момент времени upload не упал,
                # ну и вообще когда один догоняет другой,
                # думаю по размеру файла проверять (блокировать или отдавать b'')
                if self.already_read == self.file_size:
                    return buf
                else:
                    return buf + self.read(needs)


    # def read(self, chunk_size):
    #     """
    #     Функция записывает ответ _read2 в переменную last_read,
    #     в случае ошибки при работе с данными полученными от фукнции,
    #     есть возможность продолжить с того же места
    #     """
    #     if self.is_error:
    #         self.is_error = False
    #     else:
    #         self.last_read = self._read2(chunk_size)
    #     return self.last_read


    def _write(self, chunk):
        chunk_size = len(chunk)
        self.buffer.seek(self.pos_w)
        self.buffer.write(chunk)
        self.pos_w += chunk_size
        self.already_wrote += chunk_size


    def write(self, chunk):
        chunk_size = len(chunk)

        with self.threads_lock:

            available = 0

            if self.pos_w >= self.pos_r:
                available = self.buf_size - self.pos_w
            else:
                available = self.pos_r - self.pos_w

            if available >= chunk_size:
                self._write(chunk)
            else:
                self._write(chunk[0, available-1])
                # не будет ли здесь взаимной блокировки?
                self.write(chunk[available, chunk_size-1])


    def __del__(self):
        # save to flash
        self.buffer.close()


    def ram_to_flash(self, local_path):
        """
        Должен использовать в случае исключений при прерывании
        Или при переходе на хранения всего файла во Flash

        Нужен метод для сохранения в файл local
        Этот метод должен вызваться в самом начале исполнения uploader,
        чтобы буффер не успел ничем затереться
        """
        pass


    def maximize_buffer(self):
        """
        Может работать только, если считано строго меньше размера буффера
        в обратном случае исключение
        """
        pass


    def rename_file_buffer(self, local_path):
        pass


    def is_read_all(self):
        if self.file_size == self.already_read:
            return True
        else:
            return False


    def is_wrote_all(self):
        if self.file_size == self.already_wrote:
            return True
        else:
            return False


    def get_optimize_buf_size(self):
        if self.buf_type == 0: # RAM
            # < 10MB
            if self.file_size < 10000000:
                return self.file_size
            else:
                return 10000000

        elif self.buf_type == 1: # FLASH
            # < 100MB
            if self.file_size < 100000000:
                return self.file_size
            else:
                return 100000000


    # def was_error(self):
    #     self.is_error = True


    def seekable(self):
        return False


    def seek(self, offset, whence=0):
        """
        Only for read side!

        ================================================
        Change the stream position to the given byte offset.
        offset is interpreted relative to the position indicated by whence.
        The default value for whence is SEEK_SET. Values for whence are:

        SEEK_SET or 0 – start of the stream (the default); offset should be zero or positive
        SEEK_CUR or 1 – current stream position; offset may be negative
        SEEK_END or 2 – end of the stream; offset is usually negative
        Return the new absolute position.

        New in version 2.7: The SEEK_* constants
        ================================================

        | *         +   |
        | 0 1 2 3 4 5 6 |
        |     -         |

        * - pos_s - статический размер незатераемой истории
            pos_s = pos_r - amount_of_history
            нужно выбрать аккуратно чтобы при заданных размеров
            чанка и буффера не проходила блокировка
            pos_w + chunk_w + pos_s + pos_r + chunk_r >= buf_size

        + - pos_w - позиция с которой начинается запись
        - - pos_r - позиция последнего прочитанного элемента

        offset - позиция в байтах от начала файла,
        и если она укладывается между
        + справа и + слева, то смещаем pos_r и pos_s,
        иначе raise CriticalException
        """
        if whence != 0:
            raise OSError("seek() not support relative offset")

        raise OSError("SmartBuffer doesn't support seek()")


    def tell(self):
        raise OSError("SmartBuffer doesn't support tell()")


    def truncate(self):
        """
        Resize the stream to the given size in bytes (or the current position if size is not specified).
        The current stream position isn’t changed. This resizing can extend or reduce the current file size.
        In case of extension, the contents of the new file area depend on the platform
        (on most systems, additional bytes are zero-filled). The new file size is returned.
        Changed in version 3.5: Windows will now zero-fill files when extending.
        """
        raise OSError("SmartBuffer doesn't support truncate()")


    def writable(self):
        return True


    def update_really_read_bytes(self, bytes):
        """
        [ pos_w, really_read_bytes, pos_r ]

        really_read_bytes <= aleready_read

        """
        # self.really_read = bytes
        pass


    def reset_pos_r_to_really_read_bytes(self):
        # self.pos_r = self.really_read
        pass


    # вообще, по хорошему нужен метод seek для read
    # чтобы выполнить сброс используя seek()
