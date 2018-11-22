# FileSync

This software allow to sync a group of files between devices w difficult access:

* Pixracer (mavros ftp)
* FlirDuo (external flash drive)
* website (POST requests, developed for logs.px4.io)
* FTP server

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

### MAVFTP

* For use this script bundle your PixHawk must connect to Companation Computer with USB (NOT UART!)
* Нельзя запрашивать несколько mavftp-команд параллельно, будет ошибка: "FTP: Busy". По этой причине нельзя создавать несколько downloader'ов, а также по этой причине введена mavftp_lock между downloader'ом и finder'ом. Аналогичным образом работает QGC.
* Найти ошибку в crc32 для проверки файлов по контрольной сумме
* Ускорить загрузку логов https://github.com/mavlink/mavros/issues/874

### TODO

* Понять нужны ли join() для thread'ов
* Нужнен гибкий конфигурационный файл с правилами, откуда, куда и что делать.
* Если процесс непрерывный и последовательный, можно сделать одну числовую переменную, описывающую состояния файла. Например:
  * 0 - лог найден в source
  * 1 - лог скачен в local
  * 2 - лог выкачен в target
  * 3 - лог удален из source
  * 4 - лог выкачен не полностью
