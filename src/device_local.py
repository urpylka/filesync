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

import os, time
from device_abstract import device_abstract


class device_local(device_abstract):
    """
    Use: device_local(mount_point="/")
    > mount_point is like root path for device_local or device_disk

    """

    @staticmethod
    def to_string(dic): return "device_local://" + dic["mount_point"]


    @staticmethod
    def get_fields():
        list = []
        list.append("logger")
        list.append("mount_point")
        return list

    def _connect(self):
        """
        Не нужно выполнять никаких действий
        Мб только создать папку, если ее не существует
    
        """

        self.is_remote_available.clear()
        self._prefix = "device_local: " + self.kwargs["mount_point"] + ": "

        self.logger = self.kwargs.get("logger")
        if self.logger == None:
            from logger import get_logger
            self.logger = get_logger("device_local", "device_local.log", "DEBUG")

        self.logger.info(self._prefix + "Local is available. All operations is unlock")

        self.is_remote_available.set()


    def download(self, device_path, target_stream, chunk_size=1000000):
        """
        https://docs.python.org/3/library/io.html
        https://docs.python.org/3/library/asyncio-stream.html
        https://python-scripts.com/threading

        # Если делать через smart_buffer
        # target_stream.ram_to_flash(device_path)
        # типа надо как-то подтвердить что файл сохранен
        # а потом здесь нужен нормальный такой rename =D

        """

        self.is_remote_available.wait()
        self.logger.debug(self._prefix + "Downloading " + str(device_path))
        while 1:
            self.is_remote_available.wait()
            try:

                already_sent = target_stream.tell() #  already upload wo errors
                self.logger.info(self._prefix + "Downloading " + str(device_path) + " Started w " + str(already_sent))

                with open(self.kwargs["mount_point"] + device_path, 'rb') as stream:
                    stream.seek(already_sent)

                    while 1:
                        chunk = stream.read(chunk_size)
                        if not chunk: break
                        target_stream.write(chunk)
                break

            except Exception as ex:
                self.logger.error(self._prefix + "Downloading was interrupted: " + str(ex))
                time.sleep(1)


    def upload(self, source_stream, device_path, chunk_size=1000000):
        """
        with open("file_name", 'rb') as source_stream:
            target.upload(source_stream, "device_path")

        # Если делать через smart_buffer
        # source_stream.ram_to_flash(device_path)
        # типа надо как-то подтвердить что файл сохранен

        """

        self.is_remote_available.wait()
        self.logger.info(self._prefix + "Uploading " + str(device_path))
        while 1:
            self.is_remote_available.wait()
            try:

                already_sent = self.get_size(device_path) #  already upload wo errors
                self.logger.info(self._prefix + "Started w " + str(already_sent))
                source_stream.seek(already_sent)

                with open(self.kwargs["mount_point"] + device_path, 'wb') as stream:
                    while 1:
                        chunk = source_stream.read(chunk_size)
                        if not chunk: break
                        stream.write(chunk)
                break

            except Exception as ex:
                self.logger.error(self._prefix + "Uploading was interrupted: " + str(ex))
                time.sleep(1)


    def get_list(self):

        self.is_remote_available.wait()
        my_list = []
        for rootdir, dirs, files in os.walk(self.kwargs["mount_point"]):
            for file in files:

                _path = os.path.join(rootdir, file)
                device_path = _path.replace(self.kwargs["mount_point"], '', 1)
                size = os.stat(_path).st_size

                my_list.append({"path": device_path, "size": size, "hash": ""})

        return my_list


    def get_size(self, device_path):

        while 1:
            self.is_remote_available.wait()
            try:

                response = os.path.getsize(self.kwargs["mount_point"] + device_path)
                if not response is None: return response
                else: raise Exception("Can't get file size: response is None")

            except OSError as ex:
                if ex.errno == 2:
                    # No such file or directory
                    self.logger.info(self._prefix + "File was not uploaded yet: " + str(ex))
                    return 0
                else:
                    self.logger.error(self._prefix + "Can't get file size: " + str(ex))
            except Exception as ex:
                self.logger.error(self._prefix + "" + str(ex))

            time.sleep(1)


    def mkdir(self, path):

        self.is_remote_available.wait()
        self.logger.info(self._prefix + "Creating dir: " + str(path))

        if not os.path.exists(self.kwargs["mount_point"] + path):
            os.mkdir(self.kwargs["mount_point"] + path)
            self.logger.info(self._prefix + "Dir " + str(path) + " is created")
        else:
            if os.path.isfile(self.kwargs["mount_point"] + path):
                raise Exception("Path already used by file: " + str(path))
            else:
                self.logger.info("TARGET: Dir already exist: " + str(path))


    def ls(self, dir_path):

        self.is_remote_available.wait()
        cur_dir = self.kwargs["mount_point"] + dir_path
        return os.listdir(cur_dir)


    def is_dir(self, dir_path):

        self.is_remote_available.wait()
        cur_dir = self.kwargs["mount_point"] + dir_path
        return os.path.isdir(cur_dir)


    def rename(self, old_name, new_name):

        self.is_remote_available.wait()
        self.logger.info(self._prefix + "Renaming " + str(old_name) + " to " + str(new_name))

        try:
            if os.path.isfile(self.kwargs["mount_point"] + old_name):
                os.rename(self.kwargs["mount_point"] + old_name, self.kwargs["mount_point"] + new_name)
                self.logger.info(self._prefix + "New name " + str(new_name))
            else:
                raise Exception("File doesn't exist: " + str(old_name))
        except OSError as e:  ## if failed, report it back to the user ##
            self.logger.error("TARGET: %s - %s." % (e.filename, e.strerror))
        except Exception as ex:  ## if failed, report it back to the user ##
            self.logger.error("TARGET: " + str(ex))


    def delete(self, device_path):

        self.is_remote_available.wait()
        self.logger.info(self._prefix + "Deleting " + str(device_path))

        try:
            if os.path.isfile(self.kwargs["mount_point"] + device_path):
                os.remove(self.kwargs["mount_point"] + device_path)
            else:
                raise Exception("File doesn't exist: " + str(device_path))
        except OSError as e:  ## if failed, report it back to the user ##
            self.logger.error("TARGET: %s - %s." % (e.filename, e.strerror))
        except Exception as ex:  ## if failed, report it back to the user ##
            self.logger.error("TARGET: " + str(ex))
