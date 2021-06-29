# FileSync

This software allows to sync a group of files between devices w difficult access:

* Pixracer (mavros ftp)
* FlirDuo (external flash drive)
* Websites (POST requests, developed for logs.px4.io)
* FTP servers

## Start FTP server & filesync manager

For start ftp-server use:

```bash
git clone https://github.com/urpylka/filesync
cd filesync
mkdir temp
sudo ./src/ftp_server.py
```

For start filesync manager use:

```bash
git clone https://github.com/urpylka/filesync
cd filesync
mkdir temp
sudo ./src/manager.py config.json
```

Also there are deploy scripts: `deploy-filesync.sh`, `deploy-ftp.sh`. It creates services, adds them to autostart and starts them.

## For debug

```bash
rm -f flir/db.json && clear && sudo ./src/manager.py
while :; do sleep 1; clear; ls -l Sherlock.s03e01.avi; done
```

## Config file

Some devices:

```json
"source": {
    "device_class": "DISK",
    "module_path": "device_disk",
    "args": {
        "uuid": "D1C6-0146",
        "mount_point": "/mnt"
    }
},
```

```json
"target": {
    "device_class": "FTP",
    "module_path": "device_ftp",
    "args": {
        "host": "192.168.20.131",
        "user": "test-1",
        "passwd": "passwd"
    }
},
```

```json
"source": {
    "device_class": "LOCAL",
    "module_path": "device_local",
    "args": {
        "dir": "/home/pi/worker1"
    }
},
```

```json
"target": {
    "device_class": "POST",
    "module_path": "device_post",
    "args": {
        "uploader_url": "https://logs.px4.io/upload",
        "values": {
            "description": "",
            "additional_feedback": "",
            "email": "email@test.com",
            "allow_for_analysis": false,
            "obfuscated": false
        }
    }
},
```

```json
"target": {
    "device_class": "WEBDAV",
    "module_path": "device_webdav",
    "args": {
        "host": "https://webdav.yandex.ru",
        "user": "login",
        "passwd": "passwd"
    }
},
```

## Logic model

### Model of devices

**SOURCE** -> **LOCAL** -> **TARGET**

All sources & targets must be inherited from `device_abstract.py`.
The local storage placed on computer where executing this program.

### Model of program

Program wrote by pubsub technology & consist three workers:

1. **Finder** – searching new files on **source** and adding those to DB and **DownloadQueue**.
2. **Downloader** – downloading files from **source** to **local** which contain in **DownloadQueue** and adding those to DB and **UploadQueue**.
3. **Uploader** – uploading files from **local** to **target** which contain in **UploadQueue** and adding those to DB.

### Also

* The asynchronous DB based on JSON file `json_array.py`.
* Internet/connection checker.

## Other

Other information you can find in [devnotes](./devnotes.md).
