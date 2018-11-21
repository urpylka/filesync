#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

# import rospy
import sys
import json
import ftp_download_ulog
import post_upload_ulog
import os.path
#from mavros import ftp

# rosservice call /mavros/ftp/close /fs/microsd/log/2017-10-20/10_53_02.ulg
# from mavftp import *
# do_download_local("/fs/microsd/log/2017-10-20/10_53_02.ulg","logs/10_53_02.ulg",True,False,False)

# rospy.init_node('downloader_px4_ulog')

JSON_PATH='logs/px4logs.json'
LOGS_PATH='logs'


def load_json(path):
    with open(path, 'r') as infile:
        _dict=json.load(infile)
        # надо поставить оотлов ошибки, если файла нет или в нем невалидный !"[]" json "ValueError: No JSON object could be decoded"
    infile.close()
    return _dict


def dump_json(data,path):
    with open(path, 'w') as outfile:
        json.dump(data, outfile)
    outfile.close

   
def set_true_downloaded_flag(log_path,logs_dict):
    # ищем такой же лог в словаре
    for _log in logs_dict:
        # в структуре может отсуствовать name
        if (_log['path_on_px4'] == log_path): _log['downloaded'] = True
        # отлов ошибки нужно сделать "AttributeError: 'dict' object has no attribute 'name'"

def convert_to_rpi_path(log_directory,path_on_px4):
    return log_directory + '/' + os.path.basename(os.path.dirname(path_on_px4)).replace('-','') + '_' + os.path.basename(path_on_px4).replace('_','')


def main(additional_feedback):
    logs_dict=load_json(JSON_PATH)
    for _log in logs_dict:
        while not _log['downloaded']:
            # донладим (по хорошему нужна проверка что лог уже не скачен и имя не повторяется)
            # rosrun mavros mavftp download /fs/microsd/log/2017-10-20/13_29_53.ulg 13_29_53.ulg
            #
            # pi@raspberrypi:~/urpylka_px4logs $ rosservice call /mavros/ftp/open /fs/microsd/log/2017-10-20/10_53_02.ulg 0
            # size: 1676016
            # success: True
            # r_errno: 0
            #
            # pi@raspberrypi:~/urpylka_px4logs $ rosservice call /mavros/ftp/read /fs/microsd/log/2017-10-20/10_53_02.ulg 0 1676015
            # data: []
            # success: False
            # r_errno: 5
            #
            # ошибка 9 возникает когда файл не открыт
            #
            # pi@raspberrypi:~/urpylka_px4logs $ rosservice call /mavros/ftp/read /fs/microsd/log/2017-10-20/10_53_02.ulg 0 100
            # data: [85, 76, 111, 103, 1, 18, 53, 1, 58, 201, 155, 24, 0, 0, 0, 0, 40, 0, 66, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 56, 0, 73, 15, 99, 104, 97, 114, 91, 52, 48, 93, 32, 118, 101, 114, 95, 115, 119, 53, 56, 50, 48, 98, 102, 98, 55, 57, 52, 48, 101, 99, 54, 57, 53, 55, 51, 99, 97, 101, 51]
            # success: True
            # r_errno: 0
            #
            # кароче качать можно и нужно вообще побайтно (в принципе как я понял можно уже в полете), но туповато это самому реализовывать поэтому возьму это из mavftp
            _log['path_on_rpi'] = convert_to_rpi_path(LOGS_PATH,_log['path_on_px4'])
            try:
                ftp_download_ulog.do_download_local(_log['path_on_px4'],_log['path_on_rpi'],True,True,True)
                # если успешно донладим, меняем флаг в словаре в памяти
                _log['downloaded'] = True
                # по хорошему надо раньше это сделать, но можно и здесьменяем флаг в словаре в json
                dump_json(logs_dict,JSON_PATH)
            except Exception as ex:
                # может быть ошибка что флешка на пиксе не доступна (ошибка 110 например)
                print(ex.message)
                # закрыть поток на ftp "rosservice call /mavros/ftp/close NAME_OF_FILE" & сбросить ftp "rosservice call /mavros/ftp/reset"
        if _log['downloaded']: # в принципе можно убрать, но фиг знает что будет, если ошибка вызовет break
            while not _log['uploaded']:
                try:
                    # хорошая идея, если развертывать свой logs.px4.io можно возвращаться как-то еще ссылку на удаление лога
                    status_code, log_url = post_upload_ulog.upload(os.path.basename(_log['path_on_rpi']), additional_feedback, _log['path_on_rpi'])
                    if status_code == 200:
                        print("Uploaded to " + log_url)
                        _log['uploaded'] = True
                        _log['url_px4io'] = log_url
                        # по хорошему надо раньше это сделать, но можно и здесьменяем флаг в словаре в json
                        dump_json(logs_dict,JSON_PATH)
                except Exception as ex:
                    # мб такая ошибка [Errno 2] No such file or directory: u'logs/20171019_143151.ulg'
                    # в этом случае надо сбросить _log[download] = False
                    print(str(ex))

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print ("Error: Please enter additional_feedback for your logs")
        else:
            main(sys.argv[1])
    except Exception as ex:
        print("Error: " + str(ex))
        #ftp.reset()
