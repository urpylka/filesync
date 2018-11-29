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
import os.path
import time
from threading import Thread
from queue import Queue

from json_array import JsonArray
from device_disk import DISK
from device_ftp import FTP
from logger import get_logger


logger = get_logger("filesync", "/home/pi/flir/filesync.log")
source = DISK("66F8-E5D9", "/mnt", logger)

buffer = io.BytesIO()
source.stream_download("/20181106_163024/20181106_163024_949.JPG", buffer)
buffer.close()
print(buffer.readinto())