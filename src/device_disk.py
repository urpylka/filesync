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
        self.kwargs["logger"].info("SOURCE: Partition is unavailable. All operations is lock")
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
                        self.kwargs["logger"].info("SOURCE: Partition is available. All operations is unlock")
                else:
                    self.kwargs["logger"].debug("SOURCE: Try to mount partition")
                    # http://qaru.site/questions/447799/what-happens-if-you-mount-to-a-non-empty-mount-point-with-fuse
                    # чтобы не было ошибок с exfat можно добавить флаг "-o nonempty"
                    # или, как сделал я, сначала вызвать umount
                    a0, b0, c0 = bash_command("/bin/umount " + self.kwargs["mount_point"], self.kwargs["logger"])
                    a, b, c = bash_command("/bin/mount /dev/disk/by-uuid/" + self.kwargs["uuid"] + " " + self.kwargs["mount_point"], self.kwargs["logger"])
            else:
                if self.is_remote_available.is_set():
                    self.is_remote_available.clear()
                    self.kwargs["logger"].info("SOURCE: Partition is unavailable. All operations is lock")

                    if code == 32:
                        self.kwargs["logger"].debug("SOURCE: The partition was ejected")
                    else:
                        self.kwargs["logger"].debug("SOURCE: lsblk returned code: " + str(code))


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


    def download(self, device_path, target_stream, chunk_size=8192):
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

        self.kwargs["logger"].debug("Downloading from " + str(device_path))

        downloading = 1
        while downloading:

            # вообще хорошо бы считать сколько
            # мы записали и сохранять это в бд
            # и для возвращения к загрузке брать данные из бд
            # already_save = target_stream.tell()

            self.is_remote_available.wait()
            with open(self.kwargs["mount_point"] + device_path, 'rb') as stream:

                stream.seek(target_stream.tell())
                try:
                    while 1:
                        chunk = stream.read(chunk_size)

                        if not chunk:
                            downloading = 0
                            break

                        target_stream.write(chunk)
                except Exception as ex:
                    self.kwargs["logger"].error("SOURCE: Downloading was interrupting: " + str(ex))
                    time.sleep(1)


    def get_size(self, device_path):

        while 1:
            self.is_remote_available.wait()

            try:
                self.is_remote_available.wait()
                response = os.path.getsize(device_path)

                if not response is None:
                    return response
                else:
                    raise Exception("Can't get file size: response is None")

            except Exception as ex:
                self.kwargs["logger"].error("TARGET: " + str(ex))


    def upload(self, source_stream, device_path, chunk_size=8192):
        """

        with open("file_name", 'rb') as source_stream:
            target.upload(source_stream, "device_path")

        """

        self.kwargs["logger"].info("TARGET: Uploading: " + str(device_path))

        self.is_remote_available.wait()

        while 1:
            time.sleep(1)
            self.kwargs["logger"].debug("TARGET: enter upload")

            try:
                already_sent = self.get_size(device_path) #  already upload wo errors
                self.kwargs["logger"].info("TARGET: Started w " + str(already_sent))
                source_stream.seek(already_sent)

                self.is_remote_available.wait()
                with open(self.kwargs["mount_point"] + device_path, 'wb') as stream:
                    while True:
                        chunk = source_stream.read(chunk_size)
                        if not chunk: break
                        stream.write(chunk)

            except Exception as ex:
                self.kwargs["logger"].error("TARGET: " + str(ex))
        self.kwargs["logger"].debug("TARGET: exit upload")


    def delete(self, device_path):
        self.kwargs["logger"].info("SOURCE: Deleting: " + str(device_path))

        # try:
        #     if os.path.isfile(device_path):
        #         os.remove(device_path)
        #     else:
        #         raise Exception("File doesn't exist")
        # except OSError as e:  ## if failed, report it back to the user ##
        #     self.kwargs["logger"].error("TARGET: %s - %s." % (e.filename, e.strerror))
        # except Exception as ex:  ## if failed, report it back to the user ##
        #     self.kwargs["logger"].error("TARGET: " + str(ex))
