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

#
# Description: Python script to upload log to px4 flight_review project.
#
# https://github.com/PX4/flight_review
# logs.px4.io
#
# Author: Smirnov Artem (@urpylka) 26.10.2017
#
# Link to tutorial how to used python requests
# http://docs.python-requests.org/en/master/user/quickstart/
# 
# Use: python3 uploader_ulog.py -d="id1 26102017" -f="миссия" -p=/Volume/ID1/2017-10-26/17_44_35.ulg 
#

# https://github.com/xianshidan/chunk-download/blob/master/chunk_download.py

import os, argparse, requests

def get_arguments():
    parser = argparse.ArgumentParser(description='Python script for uploading ulog to the PX4/flight_review database.', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--description', '-d', type=str, default="data/downloaded/", help='Uploader\'s description.')
    parser.add_argument('--additional-feedback', '-f', type=str, help='Uploader\'s additional feedback.')
    parser.add_argument('--email', '-e', type=str, default="i@smirart.ru", help='Uploafer\'s email.')
    parser.add_argument('--uploader-url', '-u', type=str, default="https://logs.px4.io/upload", help='URI to the PX4/flight_review database.')
    parser.add_argument('--file-path', '-p', type=str, help='Path to ulog file.')
    return parser.parse_args()


def main():
    args = get_arguments()
    status_code, log_url = upload(args.description, args.additional_feedback, args.file_path, args.email, args.uploader_url)
    print(status_code)
    print(log_url)

def upload(description, additional_feedback, file_path, email="i@smirart.ru", uploader_url="https://logs.px4.io/upload", allow_for_analysis='false', obfuscated='false'):
    """
    log_url = upload(os.path.basename(log['path_on_rpi']), _USER_FEEDBACK, log['path_on_rpi'], _USER_EMAIL, _UPLOADER_URL)
    """

    #print('description: ' + description + \
    # '\nadditional_feedback: ' + additional_feedback + \
    # '\nemail: ' + email + \
    # '\nuploader_url: ' + uploader_url + \
    # '\nfile_path: ' + file_path + \
    # '\nallow_for_analysis: ' + allow_for_analysis + \
    # '\nobfuscated: ' + obfuscated)

    try:
        files = {'filearg': open(file_path,'rb')}
        values = {'description': description, 'feedback': additional_feedback, 'email': email, 'allowForAnalysis': allow_for_analysis, 'obfuscated': obfuscated}
        r = requests.post(uploader_url, files=files, data=values)

    except Exception as ex:
        # мб такая ошибка [Errno 2] No such file or directory: u'logs/20171019_143151.ulg'
        # в этом случае надо сбросить log[download] = False
        raise ex

    #print('server_response_status_code: ' + str(r.status_code) + \
    # '\nlog\'s url: ' + r.url)

    if r.status_code == 200:
        return r.url
    else: return False


if __name__ == '__main__':
    main()
