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

Source & Target consists public `is_remote_available` event.

### Model of program

Program wrote by pubsub technology & consist three workers:

1. **Finder** – searching new files on **source** and adding those to DB and **DownloadQueue**.
2. **Downloader** – downloading files from **source** to **local** which contain in **DownloadQueue** and adding those to DB and **UploadQueue**.
3. **Uploader** – uploading files from **local** to **target** which contain in **UploadQueue** and adding those to DB.

### Also

* The asynchronous DB based on JSON file `json_array.py`.
* Internet/connection checker.

## Other

For use this script bundle your PixHawk must connect to Companation Computer with USB (NOT UART!)

### Ideas

Нужнен гибкий конфигурационный файл с правилами, откуда, куда и что делать.
Library for continue a interrupting downloads