# FileSync

This software allow to sync a group of files between devices w difficult access:

* Pixracer (mavros ftp)
* FlirDuo (external flash drive)
* website (POST requests, developed for logs.px4.io)
* FTP server

## Logic model

### Model of devices

**SOURCE** -> **LOCAL** -> **TARGET**

All sources & targets must be inherited from `source_abstract.py` & `target_abstract.py`.
The local storage placed on computer where executing this program.

### Model of program

Program wrote by pubsub technology & consist three workers:

1. **Finder** – searching new files on **source** and adding those to DB and **DownloadQueue**.
2. **Downloader** – downloading files from **source** to **local** which contain in **DownloadQueue** and adding those to DB and **UploadQueue**.
3. **Uploader** – uploading files from **local** to **target** which contain in **UploadQueue** and adding those to DB.

## Other

For use this script bundle your PixHawk must connect to Companation Computer with USB (NOT UART!)

### First structure

* **creator_list_of_logs.py** - for create db of your logs on PX4
* **ftp_download_ulog.py** - for download log from PX4
* **post_upload_ulog.py** - for upload log to https://logs.px4.io
* **manager_ulog.py** - main script of pipelining processing to download & upload logs
