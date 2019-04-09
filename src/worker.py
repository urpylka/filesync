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

from smart_buffer import SmartBuffer

def in_thread(function, *args):
    #name='Worker-1'
    t = Thread(target=function, args=(args[0], args[1], args[2],))
    t.daemon = True
    t.start()
    return t


class Worker(object):

    def __init__(self, logger, source, target):
        self.logger = logger
        self.source = source
        self.target = target

        self.logger.debug("Worker-" + str(number) + " was created.")
