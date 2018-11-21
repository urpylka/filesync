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

---

Library for continue a interrupting downloads

---

Может сделать фукнции donwload и upload с использованием потоков,
таким образом подставив file.open() можно будет писать в файл,
а если их направить друг на друга они будут писать без сохранения в local

---

Если процесс непрерывный и последовательный, можно сделать одну числовую переменную, описывающую состояния файла. Например:

* 0 - лог найден в source
* 1 - лог скачен в local
* 2 - лог выкачен в target
* 3 - лог удален из source
* 4 - лог выкачен не полностью

---

Может сделать внутреннюю блокировку в source и target тк бывают случаи когда несколько device не могут работать параллельно