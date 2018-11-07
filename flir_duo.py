#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import subprocess, os, time
from threading import Thread, Event

def _get_list_of_files(path, files_extentions, verbose = True):
    """
    Get list of files
    """
    my_list = []
    if verbose: print(path, str(files_extentions))
    for rootdir, dirs, files in os.walk(path):
        for file in files:
            if files_extentions.count(file.split('.')[-1]) == 1:
                my_list.append(os.path.join(rootdir, file))
    return my_list

def _bash_command(command, verbose = True):
    if verbose: print("Execute: " + str(command))
    try:
        do_command = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError as grepexc:                                                                                       
        print("Error code", grepexc.returncode, grepexc.output)
        return 1000, None, None

    output, error = do_command.communicate()
    retcode = do_command.returncode
    if verbose: print("RETCODE: " + str(retcode))
    if verbose: print("STDOUT: " + str(output))
    if verbose: print("STDERR: " + str(error))
    do_command.wait()
    return retcode, output, error

class FlirDuoCamera():
    """
    It can work w USB flash drive

    FlirDuoCamera class consist:
    - get_list_of_files()
    - download(remote_path, local_path)
    - del_file(remote_path)
    """
    def __init__(self, UUID, files_extentions):
        self._UUID = UUID
        self._files_extentions = files_extentions
        self.MOUNT_POINT = "/mnt"

        self.is_remote_available = Event()
        self.is_remote_available.clear()
        print("Раздел недоступен, все операции заблокированы")
        t = Thread(target = self._mount, args = ())
        t.daemon = True
        t.start()

    def get_list_of_files(self):
        return _get_list_of_files(self.MOUNT_POINT, self._files_extentions)

    def _mount(self):
        while True:
            time.sleep(1)
            code = None
            output = None
            code, output, error = _bash_command("/bin/lsblk -o MOUNTPOINT \"/dev/disk/by-uuid/" + self._UUID + "\"")
            if code == 0:
                if output.find(self.MOUNT_POINT) > -1:
                    if not self.is_remote_available.is_set():
                        self.is_remote_available.set()
                        print("Раздел доступен, все операции разблокированы")
                else:
                    a, b, c = _bash_command("/bin/mount /dev/disk/by-uuid/" + self._UUID + " " + self.MOUNT_POINT)
                    continue
            else:
                if self.is_remote_available.is_set():
                    self.is_remote_available.clear()
                    print("Раздел недоступен, все операции заблокированы")

                if code == 32:
                    print("The partition isn't found yet")
                else:
                    print("lsblk returned code: " + str(code))

    def _get_checksum_flash():
        return True

    def _get_checksum_local():
        return True

    def download(remote_path, local_path, verbose = True):
        """
        Можно реализовать проверку по размеру файла на то копировать его просто, используя cp, или чанками
        """
        if verbose: print("Downloading from " + str(remote_path) + " to " + str(local_path))
        while True:
            try:
                code, output, error = _bash_command("/bin/cp " + remote_path + " " + local_path)
                if code == 0:
                    if _get_checksum_flash(remote_path) == _get_checksum_local(local_path):
                        return True
                else:
                    print("cp returned code: " + str(code) + " and message: " + str(output))
                    time.sleep(1)
            except Exception as ex:
                raise Exception("Download error: " + str(ex))

    def del_file(file_path):
        code, output, error = _bash_command("/bin/rm " + file_path)
        if code != 0:
            print(output)
        return code