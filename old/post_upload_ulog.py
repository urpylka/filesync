#! /usr/bin/env python
# -*- coding: utf-8 -*-
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

import os
import argparse
import requests

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

    #print('description: ' + description)
    #print('additional_feedback: ' + additional_feedback)
    #print('email: ' + email)
    #print('uploader_url: ' + uploader_url)
    #print('file_path: ' + file_path)
    #print('allow_for_analysis: ' + allow_for_analysis)
    #print('obfuscated: ' + obfuscated)

    files = {'filearg': open(file_path,'rb')}
    values = {'description': description, 'feedback': additional_feedback, 'email': email, 'allowForAnalysis': allow_for_analysis, 'obfuscated': obfuscated}
    r = requests.post(uploader_url, files=files, data=values)

    #print('server_response_status_code: ' + str(r.status_code))
    #print('log\'s url: ' + r.url)

    return r.status_code, r.url

if __name__ == '__main__':
    main()
