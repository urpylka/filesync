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

from device_local import device_local
from bash_commands import *

import os, time


class device_disk(device_local):


    @staticmethod
    def to_string(dic): return "device_disk://" + dic["uuid"] + ":" + dic["mount_point"]


    @staticmethod
    def get_fields():
        list = []
        list.append("logger")
        list.append("uuid")
        list.append("mount_point")
        return list


    def _connect(self):
        self.is_remote_available.clear()
        self._prefix = self.kwargs["uuid"] + ": "

        self.logger = self.kwargs.get("logger")
        if self.logger == None:
            from logger import get_logger
            self.logger = get_logger("device_disk", "device_disk.log", "DEBUG")

        self.logger.info(self._prefix + "Partition is unavailable. All operations is lock")
        while True:
            time.sleep(1)
            code = None
            output = None
            code, output, error = bash_command("/bin/lsblk -o MOUNTPOINT \"/dev/disk/by-uuid/" + self.kwargs["uuid"] + "\"", self.logger)
            if code == 0:
                #if output.find((self.kwargs["mount_point"]) > -1:
                if self.kwargs["mount_point"] in str(output):
                    if not self.is_remote_available.is_set():
                        self.is_remote_available.set()
                        self.logger.info(self._prefix + "Partition is available. All operations is unlock")
                else:
                    self.logger.debug(self._prefix + "Try to mount partition")
                    # http://qaru.site/questions/447799/what-happens-if-you-mount-to-a-non-empty-mount-point-with-fuse
                    # чтобы не было ошибок с exfat можно добавить флаг "-o nonempty"
                    # или, как сделал я, сначала вызвать umount
                    a0, b0, c0 = bash_command("/bin/umount " + self.kwargs["mount_point"], self.logger)
                    a, b, c = bash_command("/bin/mount /dev/disk/by-uuid/" + self.kwargs["uuid"] + " " + self.kwargs["mount_point"], self.logger)
                    if a == 32 and c == b"mount: unknown filesystem type 'exfat'\n":
                        self.logger.error(self._prefix + "OS doesn't support exfat filesystem.\nExecute: sudo apt-get install exfat-fuse exfat-utils")
            else:
                if self.is_remote_available.is_set():
                    self.is_remote_available.clear()
                    self.logger.info(self._prefix + "Partition is unavailable. All operations is lock")

                    if code == 32:
                        self.logger.debug(self._prefix + "The partition was ejected")
                    else:
                        self.logger.debug(self._prefix + "lsblk returned code: " + str(code))


    def __del__(self):
        a0, b0, c0 = bash_command("/bin/umount " + self.kwargs["mount_point"], self.logger)


    def download_bash_local(self, remote_path, local_path):
        self.is_remote_available.wait()
        return copy(remote_path, local_path, self.logger)
