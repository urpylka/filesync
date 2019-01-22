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
import time
import sys
from threading import Lock
from threading import Thread

# http://qaru.site/questions/81837/python-ftp-chunk-iterator-without-loading-entire-file-into-memory


def in_thread(function, args):
    t = Thread(target=function, args=(args,))
    t.daemon = True
    t.start()


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

    debug = 0

    down_speed = 0
    up_speed = 0
    down_percent = 0
    up_percent = 0
    prog = 1

    def measure_down_speed(self, delay):
        diff = 0
        while self.prog:
            diff = self.alr_wrote
            time.sleep(delay)
            diff -= self.alr_wrote
            self.down_speed = -diff / delay


    def measure_up_speed(self, delay):
        diff = 0
        while self.prog:
            diff = self.alr_read
            time.sleep(delay)
            diff -= self.alr_read
            self.up_speed = -diff / delay


    def show_progress(self, delay):
        while self.prog:
            # по хорошему нужно выводить имя файла
            self.down_percent = int(self.alr_wrote / self.file_size * 100)
            self.up_percent = int(self.alr_read / self.file_size * 100)
            #http://qaru.site/questions/28009/print-to-the-same-line-and-not-a-new-line-in-python
            print(
                "DOWN [{1:12}/{2:12}] {0:3}%  {3:8}KB/s".format(self.down_percent, self.alr_wrote, self.file_size, int(self.down_speed/1000)) + \
                " <=> " + \
                "UP [{1:12}/{2:12}] {0:3}%  {3:8}KB/s".format(self.up_percent, self.alr_read, self.file_size, int(self.up_speed/1000))
                , end='\r')

            time.sleep(delay)


    def measure_progress(self):
        in_thread(self.measure_down_speed, 1)
        in_thread(self.measure_up_speed, 1)
        in_thread(self.show_progress, 1)


    def __init__(self, file_size, logger, buf_type=0, buf_size=None, buf_name=None):
        """
        Buffer can be placed in memory or flash

        если указать размер буффера равный размеру файла,
        тип буффера - файл
        и имя файла куда мы хотим его сохранить,
        то получится local
        """

        self._prefix = "SB: "
        self.logger = logger

        self.file_size = file_size
        self.buf_type = buf_type
        self.hist_size = 1000000 if file_size > 1000000 else file_size # min(1MB, file_size)

        if buf_size is None:
            self.buf_size = self.get_optimize_buf_size()
        else:
            self.buf_size = buf_size

        if buf_type == 0:
            self.buffer = io.BytesIO()
        elif buf_type == 1:
            self.buffer = io.open(buf_name, "w+b")

        self.pos_r = 0
        self.pos_w = 0
        self.pos_h = self.buf_size - 1
        if self.pos_h <= 0:
            raise Exception("pos_h incorrect: " + str(self.pos_h))

        self.alr_read = 0
        self.alr_wrote = 0

        self.offset = 0 # нужно для смещения left

        self.stop_writer = False
        self.can_read = False
        self.can_write = True

        self.measure_progress()

        self.show_stat()


    def _get_left(self):
        left = self.alr_wrote - self.buf_size
        # if left < 0: left = 0
        # не нужно тк offset начинается с 0
        if left < self.offset: left = self.offset
        return left


    def _read(self, chunk_size):
        """
        Функция умеет читать только до границы буффера
        """
        self.buffer.seek(self.pos_r)
        buf = self.buffer.read(chunk_size)

        self.pos_r += chunk_size

        if self.pos_r == self.buf_size:
            self.pos_r = 0

        self.alr_read += chunk_size

        if self.alr_read > self.file_size:
            raise Exception("alr_read:" + str(self.alr_read) + " > file_size:" + str(self.file_size))

        # смещение позиции незатираемой истории
        self.pos_h = max(self._get_left(), self.alr_read - self.hist_size) % self.buf_size

        if self.debug: self.show_stat()

        return buf


    def _write(self, chunk):
        chunk_size = len(chunk)
        self.logger.debug(self._prefix + "_w: " + str(chunk_size))
        self.buffer.seek(self.pos_w)
        self.buffer.write(chunk)
        self.pos_w += chunk_size
        if self.pos_w == self.buf_size:
            self.pos_w = 0
        self.alr_wrote += chunk_size

        if self.debug: self.show_stat()

        if self.alr_read < self._get_left():
            raise Exception("alr_read:" + str(self.alr_read) + " < _get_left:" + str(self._get_left()))


    def read(self, chunk_size):
        """
        available - то, что можно в одну строну прочитать методом _read
        """
        self.logger.debug(self._prefix + "Pull: " + str(chunk_size))

        if chunk_size < 0:
            # В аналогичных функциях read chunk_size
            # может равняться -1 тогда будет весь буффер
            raise Exception("Size of the chunk must be positive")

        self.threads_lock.acquire()

        available = self.av_r()
        self.can_read = False
        self.logger.debug(self._prefix + "av_r: " + str(available))

        buf = b''

        if available >= chunk_size:
            # читаем целый чанк
            buf = self._read(chunk_size)
            self.threads_lock.release()
        else:

            if available > 0:
                buf = self._read(available)

            self.threads_lock.release()

            if not self.is_read_all():
                needs = chunk_size - available

                # возможно нужно что-то более изящное
                while self.av_r() < 1:
                    # time.sleep(8)
                    # self.logger.debug(self._prefix + "needs: " + str(needs))
                    pass

                buf += self.read(needs)

        return buf


    def av_r(self):
        """
        Available bytes for read (to pos_w or buf_size)
        w check if pos > file_size.
        """
        av = 0

        # if self.pos_w >= self.pos_r:
        #     av = self.pos_w - self.pos_r
        # else:
        #     av = self.buf_size - self.pos_r

        # can_read => pos_w == pos_r
        if self.pos_w < self.pos_r or self.can_read:
            av = self.buf_size - self.pos_r
        else:
            av = self.pos_w - self.pos_r

        # # Проблема с блокировкой после seek(), 
        # # когда позиция pos_r становится <= por_w
        # if av == 0 and self.can_read:
        #     av = self.buf_size - self.pos_r
        #     # chunk must be < buf_size (иначе уйдем за 50.000.000 при pos_r = 0)

        # self.can_read = False

        av2 = self.file_size - self.alr_read
        if av2 < 0: raise Exception("alr_read > file_size")

        if av > av2: av = av2
        return av


    def av_w(self):
        """
        Return available bytes for write (to pos_h or buf_size)
        w check if pos > file_size.
        """
        av = 0
        # if self.pos_w > self.pos_h:

        # can_write => pos_w == pos_h
        if self.pos_w > self.pos_h or self.can_write:
            av = self.buf_size - self.pos_w
        else:
            av = self.pos_h - self.pos_w

        # # Решение проблемы со сбросом сначала
        # if av == 0 and self.can_write:
        #     av = self.buf_size - self.pos_w
        #     # chunk must be < buf_size (иначе уйдем за 50.000.000 при pos_w = 0)

        # self.can_write = False

        av2 = self.file_size - self.alr_wrote
        if av2 < 0: raise Exception("alr_wrote > file_size")

        if av > av2: av = av2
        return av


    # https://python-scripts.com/synchronization-between-threads
    def write(self, chunk):
        chunk_size = len(chunk)
        self.logger.debug(self._prefix + "Push: " + str(chunk_size))

        self.threads_lock.acquire()

        # если пытаемся запихнуть больше чем размер файла
        # этому куску не откуда взяться, тк размер файла задан в буффере
        # и если мы его превысили, значит мы где-то ошиблись индексом
        # Нужно начинать сначала
        if chunk_size + self.alr_wrote > self.file_size:
            raise BufferError("chunk_size:" + str(chunk_size) + " + alr_wrote:" + str(self.alr_wrote) + " > file_size:" + str(self.file_size))

        available = self.av_w()
        self.can_write = False
        self.logger.debug(self._prefix + "av_w: " + str(available))

        if available >= chunk_size:
            self.logger.debug(self._prefix + "urpylka-w1")
            self._write(chunk)

            if self.stop_writer:
                self.stop_writer = False

            self.threads_lock.release()
        else:
            if available > 0:
                self.logger.debug(self._prefix + "urpylka-w2")
                self._write(chunk[0:available])

            self.threads_lock.release()

            while self.av_w() < 1:
                if self.stop_writer:
                    self.logger.debug(self._prefix + "Interrupting av_w()")
                    break
                else:
                    pass

            if self.stop_writer:
                self.stop_writer = False
                raise Exception("Interrupting writer from seek()")
            else:
                self.logger.debug(self._prefix + "urpylka-w3")

                # здесь available уже другой (если не было прерывания,
                # тк отпущена блокировка, точно больше нуля)
                av = self.av_w()
                self.logger.debug(self._prefix + "av_w: " + str(av))

                self.write(chunk[available:chunk_size])
                self.logger.debug(self._prefix + "urpylka-w4")


    def show_stat(self):
        self.logger.info(self._prefix + "===============================================")
        self.logger.info(self._prefix + "buf_type:\t{0:11}".format(self.buf_type))
        self.logger.info(self._prefix + "buf_size:\t{0:11}".format(self.buf_size))
        self.logger.info(self._prefix + "file_size:\t{0:11}".format(self.file_size))
        self.logger.info(self._prefix + "left:\t{0:11}".format(self._get_left()))
        self.logger.info(self._prefix + "alr_wrote:\t{0:11}".format(self.alr_wrote))
        self.logger.info(self._prefix + "alr_read:\t{0:11}".format(self.alr_read))
        self.logger.info(self._prefix + "hist_size:\t{0:11}".format(self.hist_size))
        self.logger.info(self._prefix + "pos_r:\t{0:11}".format(self.pos_r))
        self.logger.info(self._prefix + "pos_w:\t{0:11}".format(self.pos_w))
        self.logger.info(self._prefix + "pos_h:\t{0:11}".format(self.pos_h))
        self.logger.info(self._prefix + "av_write:\t{0:11}".format(self.av_w()))
        self.logger.info(self._prefix + "av_read:\t{0:11}".format(self.av_r()))
        self.logger.info(self._prefix + "===============================================")


    def is_read_all(self):
        if self.file_size == self.alr_read:
            return True
        else:
            return False


    def is_wrote_all(self):
        if self.file_size == self.alr_wrote:
            return True
        else:
            return False


    def get_optimize_buf_size(self):
        if self.buf_type == 0: # RAM
            # < 50MB
            MAX = 50000000
            if self.file_size < MAX:
                return self.file_size
            else:
                return MAX

        elif self.buf_type == 1: # FLASH
            # < 100MB
            MAX = 100000000
            if self.file_size < MAX:
                return self.file_size
            else:
                return MAX
 

    def seekable(self):
        """
        Only for read side!
        """
        return True


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

        * - pos_h - статический размер незатераемой истории
            pos_h = pos_r - amount_of_history
            нужно выбрать аккуратно чтобы при заданных размеров
            чанка и буффера не проходила блокировка
            pos_w + chunk_w + pos_h + pos_r + chunk_r >= buf_size

        + - pos_w - позиция с которой начинается запись,
        но пока там ничего не записано
        - - pos_r - позиция следующего элемента который
        будет считан, может использоваться только если pos_w ушел вперед

        offset - позиция в байтах от начала файла,
        и если она укладывается между
        + справа и + слева, то смещаем pos_r и pos_h,
        иначе raise CriticalException
        """
        self.logger.info(self._prefix + "seek() to:\t" + str(offset))

        if whence != 0:
            raise NotImplementedError("seek() doesn't support relative offset")

        if offset > self.file_size:
            raise AttributeError("Data couldn't be reached: offset > file_size")
            # работаем только,
            # если self.file_size >= offset
        
        if offset < 0:
            raise Exception("offset < 0")

        with self.threads_lock:
            # предлагаю остановить writer в любом случае сразу,
            # потому что есть av_w() исполняется без блокировки неправильно
            self.stop_writer = True

            self.alr_read = offset
            self.pos_r = self.alr_read % self.buf_size

            left = self._get_left()

            if  left <= offset <= self.alr_wrote:
                # Если мы не вышли за пределы буффера

                # смещение позиции незатираемой истории
                self.pos_h = max(left, self.alr_read - self.hist_size) % self.buf_size

                # if offset == left => pos_r = pos_w => блокировка reader
                if self.pos_r == self.pos_w:
                    self.can_read = True

            else:
                #raise BufferError("The data is already rewrite")
                #raise EOFError("The data isn't wrote yet") # writer может дописать данные

                self.offset = offset # нужно для смещения left

                self.alr_wrote = offset
                self.pos_w = self.alr_wrote % self.buf_size

                # смещение позиции незатираемой истории
                self.pos_h = self.pos_w

                # offset = left = alr_w = alr_r => pos_w = pos_h => возникает блокировка
                self.can_write = True

            self.show_stat()


    def tell(self):
        """
        Only for write side!
        """
        return self.alr_wrote


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


    def __del__(self):
        # save to flash
        self.buffer.close()
        self.prog = 0


    # def ram_to_flash(self, local_path):
    #     """
    #     Должен использовать в случае исключений при прерывании
    #     Или при переходе на хранения всего файла во Flash

    #     Нужен метод для сохранения в файл local
    #     Этот метод должен вызваться в самом начале исполнения uploader,
    #     чтобы буффер не успел ничем затереться
    #     """
    #     pass


    # def maximize_buffer(self):
    #     """
    #     Может работать только, если считано строго меньше размера буффера
    #     в обратном случае исключение
    #     """
    #     pass


    # def rename_file_buffer(self, local_path):
    #     pass
    

    # def _in_buffer(self, abs_pos):

    #     # if self.alr_wrote >= pos:
    #     #     if pos >= self.alr_wrote - self.buf_size:
    #     #         # все чотко - продолжаем
    #     #         pass
    #     #     else:
    #     #         # не хватает истории (мы сильно убежали вперед)
    #     #         pass
    #     # else:
    #     #     # нужно дозагрузить в буффер столько сколько нужно
    #     #     pass

    #     if self.alr_wrote - self.buf_size <= abs_pos <= self.alr_wrote:
    #         return 1
    #     else:
    #         return 0

    # def read2(self, chunk_size):
    #     """
    #     available - то, что можно в одну строну прочитать методом _read()
    #     """
    #     if chunk_size < 0:
    #         # В аналогичных функциях read chunk_size
    #         # может равняться -1 тогда будет весь буффер
    #         raise Exception("Размер чанка не может быть отрицательным")

    #     with self.threads_lock:
    #         self.logger.debug(self._prefix + "urpylka-r")
    #         diff = self.pos_w - self.pos_r

    #         # если pos_w >= pos_r
    #         # AND chunk_size > 0,
    #         # что, логично, то проверка
    #         # diff >= 0 не нужна
    #         if diff >= chunk_size:
    #             # читаем целый чанк
    #             return self._read(chunk_size)
    #         else:
    #             # если доступно меньше чанка
    #             needs = 0
    #             buf = b''

    #             if diff > 0:
    #                 # читаем до pos_w
    #                 buf = self._read(diff)
    #                 needs = chunk_size - diff

    #             elif diff < 0:
    #                 # читаем, что осталось на этой стороне
    #                 available = self.buf_size - self.pos_r
    #                 needs = chunk_size - available
    #                 buf = self._read(available)

    #             else:
    #                 # diff == 0
    #                 needs = chunk_size

    #             # нужно чтобы в первый момент времени upload не упал,
    #             # ну и вообще когда один догоняет другой,
    #             # думаю по размеру файла проверять (блокировать или отдавать b'')
    #             if self.alr_read == self.file_size:
    #                 return buf
    #             else:
    #                 return buf + self.read(needs)
