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

from device_abstract import Device
from bash_commands import *

import os, time

class DISK(Device):

    def _connect(self):
        self.is_remote_available.clear()
        self._prefix = self.kwargs["uuid"] + ": "
        self.kwargs["logger"].info(self._prefix + "Partition is unavailable. All operations is lock")
        while True:
            time.sleep(1)
            code = None
            output = None
            code, output, error = bash_command("/bin/lsblk -o MOUNTPOINT \"/dev/disk/by-uuid/" + self.kwargs["uuid"] + "\"", self.kwargs["logger"])
            if code == 0:
                #if output.find((self.kwargs["mount_point"]) > -1:
                if self.kwargs["mount_point"] in str(output):
                    if not self.is_remote_available.is_set():
                        self.is_remote_available.set()
                        self.kwargs["logger"].info(self._prefix + "Partition is available. All operations is unlock")
                else:
                    self.kwargs["logger"].debug(self._prefix + "Try to mount partition")
                    # http://qaru.site/questions/447799/what-happens-if-you-mount-to-a-non-empty-mount-point-with-fuse
                    # чтобы не было ошибок с exfat можно добавить флаг "-o nonempty"
                    # или, как сделал я, сначала вызвать umount
                    a0, b0, c0 = bash_command("/bin/umount " + self.kwargs["mount_point"], self.kwargs["logger"])
                    a, b, c = bash_command("/bin/mount /dev/disk/by-uuid/" + self.kwargs["uuid"] + " " + self.kwargs["mount_point"], self.kwargs["logger"])
                    if a == 32 and c == b"mount: unknown filesystem type 'exfat'\n":
                        self.kwargs["logger"].error(self._prefix + "OS doesn't support exfat filesystem.\nExecute: sudo apt-get install exfat-fuse exfat-utils")
            else:
                if self.is_remote_available.is_set():
                    self.is_remote_available.clear()
                    self.kwargs["logger"].info(self._prefix + "Partition is unavailable. All operations is lock")

                    if code == 32:
                        self.kwargs["logger"].debug(self._prefix + "The partition was ejected")
                    else:
                        self.kwargs["logger"].debug(self._prefix + "lsblk returned code: " + str(code))


    def __del__(self):
        a0, b0, c0 = bash_command("/bin/umount " + self.kwargs["mount_point"], self.kwargs["logger"])


    def get_list(self):
        """
        Get list of files
        """
        self.is_remote_available.wait()
        my_list = []
        for rootdir, dirs, files in os.walk(self.kwargs["mount_point"]):
            for file in files:
                _path = os.path.join(rootdir, file)
                path = _path.replace(self.kwargs["mount_point"], '', 1)
                size = os.stat(_path).st_size
                my_list.append({"path": path, "size": size, "hash": ""})
        return my_list


    def download_bash_local(self, remote_path, local_path):
        self.is_remote_available.wait()
        return copy(remote_path, local_path, self.kwargs["logger"])


    def download(self, device_path, target_stream, chunk_size=1000000):
        """
        Сделать отсчет по времени с момента начала
        типа так:
        Downloader-1 Start download FILE
        Downloader-1 Error on downloading FILE
        Downloader-1 Resume downloading FILE
        Downloader-1 End of downloading FILE

        from device_disk import DISK
        from logger import get_logger
        logger = get_logger("filesync", "/home/pi/flir/filesync.log")
        source = DISK("66F8-E5D9", "/mnt", logger)
        with open("20181106_163024_949.JPG", 'wb') as target_stream:
            source.download("/20181106_163024/20181106_163024_949.JPG", target_stream)
        print("OK")

        https://docs.python.org/3/library/io.html
        https://docs.python.org/3/library/asyncio-stream.html
        https://python-scripts.com/threading
        """
        self.is_remote_available.wait()
        self.kwargs["logger"].info(self._prefix + "Downloading " + str(device_path))
        while 1:
            self.is_remote_available.wait()
            try:

                already_sent = target_stream.tell() #  already upload wo errors
                self.kwargs["logger"].info(self._prefix + "Started w " + str(already_sent))

                with open(self.kwargs["mount_point"] + device_path, 'rb') as stream:
                    stream.seek(already_sent)

                    while 1:
                        chunk = stream.read(chunk_size)
                        if not chunk: break
                        target_stream.write(chunk)

                break

            except Exception as ex:
                self.kwargs["logger"].error(self._prefix + "Downloading was interrupted: " + str(ex))
                time.sleep(1)


    def upload(self, source_stream, device_path, chunk_size=1000000):
        """

        with open("file_name", 'rb') as source_stream:
            target.upload(source_stream, "device_path")

        """
        self.is_remote_available.wait()
        self.kwargs["logger"].info(self._prefix + "Uploading " + str(device_path))
        while 1:
            self.is_remote_available.wait()
            try:

                already_sent = self.get_size(device_path) #  already upload wo errors
                self.kwargs["logger"].info(self._prefix + "Started w " + str(already_sent))
                source_stream.seek(already_sent)

                with open(self.kwargs["mount_point"] + device_path, 'wb') as stream:
                    while 1:
                        chunk = source_stream.read(chunk_size)
                        if not chunk: break
                        stream.write(chunk)

                break

            except Exception as ex:
                self.kwargs["logger"].error(self._prefix + "Uploading was interrupted: " + str(ex))
                time.sleep(1)


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
                    self.kwargs["logger"].error(self._prefix + "File was not uploaded yet: " + str(ex))
                    return 0
                else:
                    self.kwargs["logger"].error(self._prefix + "Can't get file size: " + str(ex))
            except Exception as ex:
                self.kwargs["logger"].error(self._prefix + "" + str(ex))

            time.sleep(1)


    def delete(self, device_path):
        self.is_remote_available.wait()
        self.kwargs["logger"].info(self._prefix + "Deleting " + str(device_path))

        try:
            if os.path.isfile(self.kwargs["mount_point"] + device_path):
                os.remove(self.kwargs["mount_point"] + device_path)
            else:
                raise Exception("File doesn't exist")
        except OSError as e:  ## if failed, report it back to the user ##
            self.kwargs["logger"].error("TARGET: %s - %s." % (e.filename, e.strerror))
        except Exception as ex:  ## if failed, report it back to the user ##
            self.kwargs["logger"].error("TARGET: " + str(ex))


    def rename(self, old_name, new_name):
        self.is_remote_available.wait()
        self.kwargs["logger"].info(self._prefix + "Renaming " + str(old_name))

        try:
            if os.path.isfile(self.kwargs["mount_point"] + old_name):
                os.rename(self.kwargs["mount_point"] + old_name, self.kwargs["mount_point"] + new_name)
            else:
                raise Exception("File doesn't exist")
        except OSError as e:  ## if failed, report it back to the user ##
            self.kwargs["logger"].error("TARGET: %s - %s." % (e.filename, e.strerror))
        except Exception as ex:  ## if failed, report it back to the user ##
            self.kwargs["logger"].error("TARGET: " + str(ex))
