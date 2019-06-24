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

from threading import Thread
import os, time

from smart_buffer import SmartBuffer

def in_thread(function, *args):
    #name='Mover-1'
    t = Thread(target=function, args=(args[0], args[1], args[2],))
    t.daemon = True
    t.start()
    return t


class Mover(object):

    def __init__(self, logger, source, target, element, number):
        self.logger = logger
        self.source = source
        self.target = target
        self.element = element
        self.number = number

        self.local_directory = ""
        # self.local_path = self.local_directory + '/' + os.path.basename(os.path.dirname(self.element['source_path'])).replace('-', '') + '_' + os.path.basename(self.element['source_path']).replace('_', '')
        self.local_path = self.local_directory + '/' + os.path.basename(self.element['source_path'])
        # self.local_path = self.element['local_path']
        self.local_path_temp = os.path.basename(self.local_path) + ".temp"
        # self.target_path = '/' + os.path.basename(self.local_path)

        self.target_path = self.element['source_path']
        self.temp_target_path = self.target_path + ".part"


        self.logger.debug("Mover-" + str(self.number) + ": source_path " + self.element['source_path'] + " local_path " + self.local_path)

        self.buffer_stream = SmartBuffer(self.element['size'], self.logger, 0, None, self.local_path_temp)
        # если задать размер буффера меньше,
        # перезапустить программу с большим буффером (в файл),
        # то не знаю будет ли перезаписываться или какие-то еще глюки

        self.iter = 0

        self.logger.debug("Mover-" + str(self.number) + " was created.")


    def move(self):

        while not self.element['downloaded'] or not self.element['uploaded'] or not self.element['dropped']:
            
            self.iter += 1

            # не делать wq.done()
            if self.iter == 10:
                # critical
                break

            try:
                self.logger.info("Mover-" + str(self.number) + ": " + str(self.element['source_path']) + " starting mover. Iteration: " + str(self.iter))

                # self.buffer_stream.show_stat()

                d = None
                u = None

                if ( not self.element["downloaded"] or not self.buffer_stream.is_wrote_all() ) and not self.element['uploaded']:
                # может вообще убрать эту проверку
                    d = in_thread(self.source.download, self.element['source_path'], self.buffer_stream, 1000000)   # вставляет

                if not self.element["uploaded"]:
                    # timeout test
                    for times in range(20):
                        if self.buffer_stream.av_r() < 1:
                            time.sleep(0.1)
                        if times == 20:
                            raise Exception("Uploading 2 sec timeout. Smart buffer is empty.")

                    u = in_thread(self.target.upload, self.buffer_stream, self.temp_target_path, 400000) # сосёт

                if ( not self.element["downloaded"] or not self.buffer_stream.is_wrote_all() ) and not self.element['uploaded']:
                # может вообще убрать эту проверку
                # бывает так, что если чанк покрывает весь оставшийся кусок файла, но функция upload отрабатывает не успешно,
                # передача все равно считается завершенной
                    d.join()
                    self.logger.debug("Mover-" + str(self.number) + ": downloader")
                    if self.buffer_stream.is_wrote_all():
                        self.element["downloaded"] = True
                        self.element["hash_md5"] = self.buffer_stream.hash_md5.hexdigest()
                        # self.element["self.local_path"] = self.local_path
                        self.logger.info("Mover-" + str(self.number) + ": " + str(self.element['source_path']) + " was downloaded")

                if not self.element["uploaded"]:
                    u.join()
                    self.logger.debug("Mover-" + str(self.number) + ": uploader")

                    if self.buffer_stream.is_read_all():
                        self.element["uploaded"] = True
                        self.logger.info("Mover-" + str(self.number) + ": " + str(self.element['source_path']) + " was uploaded")

                if self.element["downloaded"] and self.element["uploaded"]:
                    self.target.rename(self.temp_target_path, self.target_path)
                    self.source.delete(self.element["source_path"])
                    self.element["dropped"] = True
                    self.logger.info("Mover-" + str(self.number) + ": " + str(self.element['source_path']) + " was deleted")

            except Exception as ex:
                self.logger.error("Mover-" + str(self.number) + ": " + str(ex) + " with file " + self.element['source_path'])
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE"
                # & сбросить ftp "rosservice call /mavros/ftp/reset"    
                # вообще, в случае этой ошибки можно перейти к другому элементу из очереди
                time.sleep(2)

        del(self.buffer_stream)

        if self.iter == 10:
            self.logger.error("Mover-" + str(self.number) + ": Couldn't correct execute mover with file " + self.element['source_path'])
        else:
            # https://docs.python.org/2/library/logging.html
            self.logger.log(41, "Mover-" + str(self.number) + ": 🍺 File '" + self.element['source_path'] + "' has been moved")
