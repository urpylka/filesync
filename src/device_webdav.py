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

import io
import os
import time
from threading import RLock

# from re import sub

try:
    from urllib.parse import unquote
except ImportError:
    from urllib import unquote

from io import BytesIO
import lxml.etree as etree
import pycurl

import webdav.client as wc
from webdav.urn import Urn
from webdav.connection import *
from webdav.exceptions import *

from device_abstract import Device
from bash_commands import *


class device_mavftp(Device):
    """
    В классе не поддерживается восстановление передачи с ее прерывания

from device_webdav import device_mavftp
args = {
'host': "https://webdav.yandex.ru",
'user':    "",
'passwd': ""}

wd = device_mavftp(**args)

wd.get_list("/")

filename = "screen.png"

device_path = "/" + filename

with open(filename, 'wb') as target_stream:
    wd.download(device_path, target_stream)

    """

    _internal_lock = RLock()
    _internal_lock2 = RLock()
    _wc = None
    _webdav_timeout = 0.5


    def _connect(self):

        self.is_remote_available.clear()
        self._prefix = self.kwargs["user"] + "@" + self.kwargs["host"] + ": "

        self.logger = self.kwargs.get("logger")
        if self.logger == None:
            from logger import get_logger
            self.logger = get_logger("device_mavftp", "device_webdav.log", "DEBUG")

        self.logger.info(self._prefix + "WEBDAV is unavailble. All operations is lock")

        options = {
        'webdav_hostname': self.kwargs["host"],
        'webdav_login':    self.kwargs["user"],
        'webdav_password': self.kwargs["passwd"]
        }

        # options = {
        # 'proxy_hostname':  "http://127.0.0.1:8080",
        # 'proxy_login':     "p_login",
        # 'proxy_password':  "p_password",
        # 'cert_path':       "/etc/ssl/certs/certificate.crt",
        # 'key_path':        "/etc/ssl/private/certificate.key",
        # 'recv_speed' : 3000000,
        # 'send_speed' : 3000000,
        # 'verbose'    : True
        # }

        try:
            self._wc = wc.Client(options)
        except:
            self.logger.error(self._prefix + "Import error.\nExecute: pip install webdavclient")

        if not self.is_remote_available.is_set():
            self.is_remote_available.set()
            self.logger.info(self._prefix + "WEBDAV is availble. All operations is unlock")


    def __del__(self):
        self._wc.__del__()


    def download(self, device_path, target_stream, chunk_size=8192):

        with self._internal_lock:
            self.is_remote_available.wait()
            self.logger.debug(self._prefix + "Downloading " + str(device_path))
            while 1:
                self.is_remote_available.wait()
                already_sent = 0
                # всегда начинаем сначала
                # нет tell()
                try:
                    self._wc.download_to(target_stream, device_path)
                    break

                except Exception as ex:
                    self.logger.error(self._prefix + "Downloading was interrupted: " + str(ex))
                    time.sleep(1)


    def upload(self, source_stream, device_path, chunk_size=8192):

        with self._internal_lock:
            self.is_remote_available.wait()
            self.logger.debug(self._prefix + "Uploading " + str(device_path))
            
            iter = 0

            while 1:
                self.is_remote_available.wait()

                already_sent = 0
                # всегда начинаем сначала
                try:
                    source_stream.seek(already_sent)
                    # self._wc.upload_from(source_stream, device_path)
                    try:
                        urn = Urn(device_path)

                        if urn.is_dir():
                            raise OptionNotValid(name="device_path", value=device_path)

                        if not self._wc.check(urn.parent()):
                            raise RemoteParentNotFound(urn.path())

                        url = {'hostname': self._wc.webdav.hostname, 'root': self._wc.webdav.root, 'path': urn.quote()}
                        options = {
                            'URL': "{hostname}{root}{path}".format(**url),
                            'HTTPHEADER': self._wc.get_header('upload_from'),
                            'UPLOAD': 1,
                            'READFUNCTION': source_stream.read,
                        }

                        request = self._wc.Request(options=options)

                        request.perform()

                        code = request.getinfo(pycurl.HTTP_CODE)
                        if str(code) == "507":
                            # raise NotEnoughSpace()
                            raise Exception("Not enough space on the server")
                        request.close()

                    except pycurl.error:
                        raise NotConnection(self._wc.webdav.hostname)

                    break

                except Exception as ex:
                    iter += 1
                    if iter == 3:
                        self.logger.error(self._prefix + "Uploading was interrupted " + str(iter) + " times: " + str(ex))
                        break
                    else:
                        self.logger.error(self._prefix + "Uploading was interrupted " + str(iter) + " times: " + str(ex))
                        time.sleep(1)


    def get_size(self, device_path):
        """
        Частое исполенение этого метода приводит к ошибке: "Method info not supported for https://webdav.yandex.ru"
        """

        while 1:
            with self._internal_lock:
                self.is_remote_available.wait()

                try:
                    start_time = time.clock()
                    size = self._wc.info(device_path)["size"]
                    self.logger.debug(self._prefix + "Time for get size of {} from server: {:g} s".format(device_path, time.clock() - start_time))
                    return size

                except Exception as ex:
                    
                    exc = str(ex)

                    if exc.startswith("Remote resource: ") and exc.endswith(" not found"):
                        self.logger.debug(self._prefix + "Can't get file size on webdav server: " + exc)
                        return 0
                    elif exc.startswith("Method info not supported for https://webdav.yandex.ru"):
                        self.logger.debug(self._prefix + "Can't get file " + device_path + " size on webdav server: " + exc)
                        time.sleep(self._webdav_timeout)
                    else:
                        raise Exception(self._prefix + "Can't get file size on webdav server: " + exc)


    def get_list(self, rootdir='/'):
        with self._internal_lock2:
            self.is_remote_available.wait()


            def recursive_files(catalog, get_list, is_dir=None):

                def slash_is_dir(path):
                    return path.endswith('/')

                if is_dir is None:
                    is_dir = slash_is_dir

                def recursive(catalog, get_list, is_dir, files, dirs=[]):
                    for item in get_list(catalog):
                        item = os.path.join(catalog, item)
                        if not is_dir(item): files.append(item)
                        else:
                            dirs.append(item)
                            recursive(item, get_list, is_dir, files, dirs)

                files = []
                recursive(catalog, get_list, is_dir, files)
                return files

            def list(remote_path='/'):
                def parse(response):

                    try:
                        response_str = response.getvalue()
                        tree = etree.fromstring(response_str)
                        hrees = [unquote(hree.text) for hree in tree.findall(".//{DAV:}href")]
                        return [Urn(hree) for hree in hrees]
                    except etree.XMLSyntaxError:
                        return list()

                try:
                    directory_urn = Urn(remote_path, directory=True)

                    # if directory_urn.path() != self._wc.root:
                    #     if not self._wc.check(directory_urn.path()):
                    #         raise RemoteResourceNotFound(directory_urn.path())

                    response = BytesIO()

                    url = {'hostname': self._wc.webdav.hostname, 'root': self._wc.webdav.root, 'path': directory_urn.quote()}
                    options = {
                        'URL': "{hostname}{root}{path}".format(**url),
                        'CUSTOMREQUEST': self._wc.requests['list'],
                        'HTTPHEADER': self._wc.get_header('list'),
                        'WRITEDATA': response,
                        'NOBODY': 0
                    }

                    request = self._wc.Request(options=options)

                    request.perform()
                    request.close()

                    urns = parse(response)

                    path = "{root}{path}".format(root=self._wc.webdav.root, path=directory_urn.path())
                    return [urn.filename() for urn in urns if urn.path() != path and urn.path() != path[:-1]]

                except pycurl.error:
                    raise NotConnection(self._wc.webdav.hostname)


            start_time = time.clock()
            print(start_time)
            # files = recursive_files(rootdir, self._wc.list)
            files = recursive_files(rootdir, list)
            my_list = []

            for path in files:
                while 1:
                    # info = self._wc.info(path)

                    info = None
                    try:
                        urn = Urn(path)
                        response = BytesIO()
                        
                        # Disable of exist check
                        #
                        # if not self._wc.check(urn.path()) and not self._wc.check(Urn(path, directory=True).path()):
                        #     raise RemoteResourceNotFound(path)

                        url = {'hostname': self._wc.webdav.hostname, 'root': self._wc.webdav.root, 'path': urn.quote()}
                        options = {
                            'URL': "{hostname}{root}{path}".format(**url),
                            'CUSTOMREQUEST': self._wc.requests['info'],
                            'HTTPHEADER': self._wc.get_header('info'),
                            'WRITEDATA': response,
                            'NOBODY': 0
                        }

                        request = self._wc.Request(options=options)

                        while 1:

                            try:
                                # time.sleep(0.3)
                                request.perform()
                            except pycurl.error as ex:
                                # raise NotConnection(self._wc.webdav.hostname)
                                self.logger.info(self._prefix + " pycurl.error returned: " + str(ex))
                                # time.sleep(self._webdav_timeout)
                                continue

                            # http://pycurl.io/docs/dev/curlobject.html
                            http_code = request.getinfo_raw(pycurl.HTTP_CODE)

                            if http_code != 207:
                                self.logger.debug(self._prefix + "Info request returned http-code: " + str(http_code))
                                time.sleep(self._webdav_timeout)
                            else:
                                request.close()
                                break

                        remote_path = "{root}{path}".format(root=self._wc.webdav.root, path=urn.path())

                        try:
                            response_str = response.getvalue()
                            # print(response_str)
                            tree = etree.fromstring(response_str)
                            find_attributes = {
                                'created': ".//{DAV:}creationdate",
                                'name': ".//{DAV:}displayname",
                                'size': ".//{DAV:}getcontentlength",
                                'modified': ".//{DAV:}getlastmodified"
                            }

                            resps = tree.findall("{DAV:}response")

                            for resp in resps:
                                href = resp.findtext("{DAV:}href")
                                urn = unquote(href)

                                if path[-1] == Urn.separate:
                                    if not remote_path == urn:
                                        continue
                                else:
                                    path_with_sep = "{path}{sep}".format(path=path, sep=Urn.separate)
                                    if not remote_path == urn and not path_with_sep == urn:
                                        continue

                                my_info = dict()
                                for (name, value) in find_attributes.items():
                                    my_info[name] = resp.findtext(value)
                                info = my_info
                                break

                            # raise RemoteResourceNotFound(remote_path)
                        except etree.XMLSyntaxError:
                            raise MethodNotSupported(name="info", server=self._wc.webdav.hostname)

                        size = int(info["size"])
                        created = info["created"]
                        modified = info["modified"]
                        my_list.append({"path": path, "size": size, "hash": "", "created": created, "modified": modified})
                        break
                    except Exception as ex:
                        exc = str(ex)

                        if exc.startswith("Remote resource: ") and exc.endswith(" not found"):
                            raise Exception(self._prefix + "Can't get info of file " + str(path) + " on webdav server: " + exc)
                        # elif exc.startswith("Method info not supported for https://webdav.yandex.ru"):
                        #     self.logger.debug(self._prefix + "Can't get 2 info of file " + str(path) + " on webdav server: " + exc)
                        #     time.sleep(self._webdav_timeout)
                        else:
                            raise Exception(self._prefix + "Can't get info of file " + str(path) + " on webdav server: " + exc)

            end_time = time.clock()
            self.logger.debug(self._prefix + "Time for get list of {:g} elements from server: {:g} s".format(len(files), end_time - start_time))

            return my_list


    def rename(self, device_path_from, device_path_to):
        """
        может все rename переименовать в move
        """

        with self._internal_lock:
            self.is_remote_available.wait()
            self.logger.info(self._prefix + "Renaming " + str(device_path_from) + " to "+ str(device_path_to))

            iter = 0
            while 1:
                self.is_remote_available.wait()
                try:
                    self._wc.move(device_path_from, device_path_to)
                    break

                except Exception as ex:
                    iter += 1
                    if iter == 3:
                        raise Exception(self._prefix + "Renaming was interrupted " + str(iter) + " times: " + str(ex))
                    else:
                        self.logger.error(self._prefix + "Renaming was interrupted " + str(iter) + " times: " + str(ex))
                        time.sleep(1)

                
                      


    def delete(self, device_path):

        with self._internal_lock:
            self.is_remote_available.wait()
            self.logger.info(self._prefix + "Deleting " + str(device_path))

            while 1:
                self.is_remote_available.wait()
                try:
                    self._wc.clean(device_path)

                    break

                except Exception as ex:
                    self.logger.error(self._prefix + "Deleting was interrupted: " + str(ex))
                    time.sleep(1)
