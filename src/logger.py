#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import logging

def get_logger(path):
    # Logger
    # https://python-scripts.com/logging-python
    logger = logging.getLogger("filesync")
    #logger.setLevel(logging.DEBUG)
    logger.setLevel(logging.INFO)
    # create the logging file handler
    fh = logging.FileHandler(path, "w", "UTF-8")
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    fh.setFormatter(formatter)
    # add handler to logger object
    logger.addHandler(fh)
    return logger

if __name__ == '__main__':
    print("logger.py is not executable")
