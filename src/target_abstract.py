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

from threading import Event

class Target(object):

    is_remote_available = Event()

    def __init__(self, *args):
        raise NotImplementedError()

    def upload(self, local_path, remote_path):
        """
        Must return bool
        """
        raise NotImplementedError()

    # Может лучше сделать Grabber or Deleter
    # def del_source_file(self, remote_path, local_path, verbose = True):
    #     pass
