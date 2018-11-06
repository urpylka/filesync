#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:set ts=4 sw=4 et:

import subprocess, os

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
        self.is_remote_available.set()
        t = Thread(target = self._mount, args = ())
        t.daemon = True
        t.start()

    def get_list_of_files(self):
        _get_list_of_files(self.MOUNT_POINT, self._files_extentions)

    def _get_list_of_files(path, files_extentions):
        """
        Get list of files
        """
        for rootdir, dirs, files in os.walk(path):
            for file in files:
                if files_extentions.count(file.split('.')[-1]) == 1:
                    return os.path.join(rootdir, file)

    def _bash_command(command):
        print("Execute: " + command)
        try:
            do_command = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        except subprocess.CalledProcessError as grepexc:                                                                                                   
            print("Error code", grepexc.returncode, grepexc.output)
            return 1, None
        if do_command.returncode == 0:
            output, error = do_command.communicate()
            return 0, output
        else:
            print("Can't exec command " + command)
            return 1, None

    def _mount(self):
        while True:
            code, output = _bash_command("/bin/lsblk -o MOUNTPOINT \"/dev/disk/by-uuid/" + self._UUID + "\" | awk '{if(NR>1) print $1;}'")
            if code == 0:
                if output == self.MOUNT_POINT:
                    if not self.is_remote_available.is_set():
                        self.is_remote_available.set()
                        print("Раздел доступен все операции разблокированы")
                else:
                    mount_code, mount_output = _bash_command("/sbin/mount /dev/disk/by-uuid/" + self._UUID + " " + self.MOUNT_POINT)
                    continue
            else:
                if self.is_remote_available.is_set():
                    self.is_remote_available.clear()
                    print("Раздел недоступен все операции заблокированы")

                if code == 32:
                    time.sleep(1)
                else:
                    print("lsblk returned code: " + code)

    def _get_checksum_flash():
        return True

    def _get_checksum_local():
        return True

    def download(remote_path, local_path):
        """
        Можно реализовать проверку по размеру файла на то копировать его просто, используя cp, или чанками
        """
        verbose = True
        print_if(verbose, "Downloading from", remote_path, "to", local_path)
        while True:
            try:
                code, output = _bash_command("/sbin/cp " + remote_path + " " + local_path)
                if code == 0:
                    if _get_checksum_flash(remote_path) == _get_checksum_local(local_path):
                        return True
                else:
                    print("cp returned code: " + code + " and message: " + output)
                    time.sleep(1)
            except Exception as ex:
                raise Exception("Download error: " + str(ex))

    def del_file(file_path):
        code, output = _bash_command("/sbin/rm " + file_path)
        if code != 0:
            print(output)
        return code