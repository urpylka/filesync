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

def get_logger(name, path, level):
    # https://python-scripts.com/logging-python
    logger = logging.getLogger(name)

    if level == "INFO":
        logger.setLevel(logging.INFO)
    elif level == "DEBUG":
        logger.setLevel(logging.DEBUG)
    elif level == "ERROR":
        logger.setLevel(logging.ERROR)

    # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(message)s")
    console_formatter = logging.Formatter("%(levelname)s - %(message)s")

    # create the logging file handler
    file_handle = logging.FileHandler(path, "w", "UTF-8")
    file_handle.setFormatter(file_formatter)

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(console_formatter)

    # add handler to logger object
    logger.addHandler(file_handle)
    logger.addHandler(console_handler)

    return logger
