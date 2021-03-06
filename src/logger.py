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

import logging

# Redirecting print to logger
# https://stackoverflow.com/questions/11124093/redirect-python-print-output-to-logger

# https://docs.python.org/3/howto/logging.html
# https://aykutakin.wordpress.com/2013/08/06/logging-to-console-and-file-in-python/

def get_logger(name, level, console_flag, path=None):
    # https://python-scripts.com/logging-python
    logger = logging.getLogger(name)

    if level == "INFO":
        logger.setLevel(logging.INFO)
    elif level == "DEBUG":
        logger.setLevel(logging.DEBUG)
    elif level == "ERROR":
        logger.setLevel(logging.ERROR)

    # https://stackoverflow.com/questions/2183233/how-to-add-a-custom-loglevel-to-pythons-logging-facility
    logging.addLevelName(41, "SUCCESS")

    if not path == "":
        # Haven't any checks from oversize!!!
        # create the logging file handler
        file_formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
        file_handle = logging.FileHandler(path, "w", "UTF-8")
        file_handle.setFormatter(file_formatter)
        logger.addHandler(file_handle)

    if console_flag:
        # console_formatter = logging.Formatter(name + ": %(levelname)s - %(message)s")
        console_formatter = logging.Formatter("%(levelname)s - %(message)s")
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)

    return logger
