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

import subprocess, os, time, logging
from threading import Thread, Event

def bash_command(command, logger=logging):
    try:
        do_command = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError as grepexc:
        logger.error("bash_command: Execute: " + str(command) + \
        "\nError code", grepexc.returncode, grepexc.output)
        return 1000, None, None

    output, error = do_command.communicate()
    retcode = do_command.returncode
    logger.debug("bash_command: Execute: " + str(command) + \
    "\nRETCODE: " + str(retcode) + \
    "\nSTDOUT: " + str(output) + \
    "\nSTDERR: " + str(error))
    do_command.wait()
    return retcode, output, error


def copy(remote_path, local_path, logger):
    """
    Можно реализовать проверку по размеру файла на то копировать его просто, используя cp, или чанками
    """
    logger.debug("copy: Downloading from " + str(remote_path) + " to " + str(local_path))
    while True:
        try:
            code, output, error = bash_command("/bin/cp " + str(remote_path) + " " + str(local_path),  logger)
            if code == 0:
                #if _get_checksum_flash(remote_path) == _get_checksum_local(local_path):
                if True:
                    return True
            else:
                logger.error("copy: cp returned code: " + str(code) + " and message: " + str(output))
                time.sleep(1)
        except Exception as ex:
            raise Exception("copy: " + str(ex))


def delete(file_path):
    code, output, error = bash_command("/bin/rm " + file_path)
    if code != 0:
        raise Exception("Не получилось удалить файл ошибка: " + output)
    return True


class DISK(Device):
    is_remote_available = Event()

    def __init__(self, *args):
        self._uuid, self._mount_point, self._logger = args

        t = Thread(target=self._mount, args=())
        t.daemon = True
        t.start()


    def get_list(self):
        """
        Get list of files
        """
        self.is_remote_available.wait()
        my_list = []
        for rootdir, dirs, files in os.walk(self._mount_point):
            for file in files:
                my_list.append(os.path.join(rootdir, file))
        return my_list


    def download(self, remote_path, local_path):
        self.is_remote_available.wait()
        return copy(remote_path, local_path, self._logger)


    def _mount(self):
        self.is_remote_available.clear()
        self._logger.info("SOURCE: Раздел недоступен, все операции заблокированы")
        while True:
            time.sleep(1)
            code = None
            output = None
            code, output, error = bash_command("/bin/lsblk -o MOUNTPOINT \"/dev/disk/by-uuid/" + self._uuid + "\"", self._logger)
            if code == 0:
                #if output.find((self._mount_point) > -1:
                if self._mount_point in str(output):
                    if not self.is_remote_available.is_set():
                        self.is_remote_available.set()
                        self._logger.info("SOURCE: Раздел доступен, все операции разблокированы")
                else:
                    self._logger.debug("SOURCE: Try to mount partition")
                    a, b, c = bash_command("/bin/mount /dev/disk/by-uuid/" + self._uuid + " " + self._mount_point, self._logger)
                    continue
            else:
                if self.is_remote_available.is_set():
                    self.is_remote_available.clear()
                    self._logger.info("SOURCE: Раздел недоступен, все операции заблокированы")

                    if code == 32:
                        self._logger.debug("SOURCE: The partition was ejected")
                    else: self._logger.debug("SOURCE: lsblk returned code: " + str(code))


    def stream_download(self, device_path, target_stream, chunk_size=1024):
        """

        target_stream = io.BytesIO()
        DISK.stream_download("device_path", target_stream)

        with open("file_name", 'wb') as target_stream:
            DISK.stream_download("device_path", target_stream)

        https://docs.python.org/3/library/io.html
        https://docs.python.org/3/library/asyncio-stream.html
        https://python-scripts.com/threading
        """
        self.is_remote_available.wait()
        self._logger.debug("Downloading from " + str(device_path))

        with open(self._mount_point + device_path, 'rb') as stream:
            while True:
                chunk = stream.read(chunk_size)
                if not chunk:
                    break
                target_stream.write(chunk)


    def stream_upload(self, source_stream, device_path, chunk_size=1024):
        """

        source_stream = io.BytesIO()
        DISK.stream_upload(source_stream, "device_path")

        with open("file_name", 'rb') as source_stream:
            DISK.stream_upload(source_stream, "device_path")

        """
        self.is_remote_available.wait()
        self._logger.debug("Upload to " + str(device_path))

        with open(self._mount_point + device_path, 'wb') as stream:
            while True:
                chunk = source_stream.read(chunk_size)
                if not chunk:
                    break
            stream.write(chunk)
