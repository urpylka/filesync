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

def load_param(param, default=None):
    if rospy.has_param(param):
        return rospy.get_param(param)
    elif not default is None:
        print("Param: " + str(param) + " not set & use default value: " + str(default))
        return rospy.get_param(param, default)
    else:
        print("Error: " + str(param) + " not set & have not default value")
        raise SystemExit


def main():
    rospy.init_node('px4logs_manager')

    USER_EMAIL = load_param('~user_email')
    USER_FEEDBACK = load_param('~user_feedback') #"Error: Please enter additional_feedback for your logs"
    FINDER_INTERVAL = load_param('~finder_interval', 10)
    ARMING_PROTECT = load_param('~arming_protect', False)
    DOWNLOADERS_COUNT = load_param('~downloaders_count', 1) # [Errno 24] Too many open files
    UPLOADERS_COUNT = load_param('~uploaders_count', 2)
    CHECK_FILE_CHECKSUM = load_param('~check_file_checksum', False)
    LOGS_DIRECTORY = load_param('~logs_directory')
    DB_JSON_PATH = load_param('~db_json_path')
    UPLOADER_URL = load_param('~uploader_url', "https://logs.px4.io/upload")
