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

### MAVFTP

* For use this script bundle your PixHawk must connect to Companation Computer with USB (NOT UART!)
* Нельзя запрашивать несколько mavftp-команд параллельно, будет ошибка: "FTP: Busy". По этой причине нельзя создавать несколько downloader'ов, а также по этой причине введена mavftp_lock между downloader'ом и finder'ом. Аналогичным образом работает QGC.

### TODO

* Найти ошибку в crc32 для проверки файлов по контрольной сумме
* Понять нужны ли join() для thread'ов
* Загрузка чанками requests https://stackoverflow.com/questions/13909900/progress-of-python-requests-post
* Добавление сервисов для работы с внутренней базой данных
* Поэксперементировать с относительнымы путями для директории логов
* Ускорить загрузку логов https://github.com/mavlink/mavros/issues/874
* Замена json_database на http://tinydb.readthedocs.io/en/latest/ (НЕ НУЖНО тк TinyDB не асинхронная)
* Брать описание для логов с /etc/coex.conf

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
