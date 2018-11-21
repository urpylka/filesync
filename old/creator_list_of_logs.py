#!/usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import rospy
import json
from mavros_msgs.srv import FileList

rospy.init_node('creator_list_px4_ulog')

JSON_PATH='logs/px4logs.json'

# /mavros/ftp/checksum
# /mavros/ftp/close
# /mavros/ftp/list
# /mavros/ftp/mkdir
# /mavros/ftp/open
# /mavros/ftp/read
# /mavros/ftp/remove
# /mavros/ftp/rename
# /mavros/ftp/reset
# /mavros/ftp/rmdir
# /mavros/ftp/truncate
# /mavros/ftp/write

# pi@raspberrypi:~ $ rosservice  
# args  call  find  info  list  type  uri

# флешки, нет в px
# pi@raspberrypi:~ $ rosservice call /mavros/ftp/list /fs/microsd/log
# list: []
# success: True
# r_errno: 0

ftp_list_call = rospy.ServiceProxy('/mavros/ftp/list', FileList)
# rosservice call /mavros/ftp/list /fs/microsd/log

def is_log_in_dict(log_path,logs_dict):
    # ищем такой же лог в словаре
    for _log in logs_dict:
        # в структуре может отсуствовать name
        if (_log['path_on_px4'] == log_path): return True
        # отлов ошибки нужно сделать "AttributeError: 'dict' object has no attribute 'name'"
    return False
    
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

def create_list():
    logs_dict=load_json(JSON_PATH)
    
    # если type равен 0 значит это файл, если 1 то это папка
    # тк пикс не создает супервложенных директорий, можно обойтись без рекурсивного перехода между папками
    main_logs_folder=ftp_list_call('/fs/microsd/log')
    if(main_logs_folder.success and main_logs_folder.r_errno == 0):
        for main_logs_folder_list in main_logs_folder.list:
            if (main_logs_folder_list.type == 1):
                path_current_folder_of_logs="/fs/microsd/log/" + main_logs_folder_list.name
                # проход по папкам с файлами логов
                current_folder_of_logs=ftp_list_call(path_current_folder_of_logs)
                if(current_folder_of_logs.success and current_folder_of_logs.r_errno == 0):
                    for log in current_folder_of_logs.list:
                        if (log.type == 0): # если найден файл лога
                            _log_path = path_current_folder_of_logs + "/" + log.name
                            if not is_log_in_dict(_log_path,logs_dict):
                                logs_dict.append({"path_on_px4":_log_path,"path_on_rpi":"","downloaded":False,"uploaded":False,"url_px4io":""})
                else: print("Import log folder error in " + path_current_folder_of_logs)
    else: print("Import log folder error in /fs/microsd/log")
    dump_json(logs_dict,JSON_PATH)

def main():
    create_list()
    logs_dict=load_json(JSON_PATH)
    print logs_dict

if __name__ == '__main__':
    main()
    