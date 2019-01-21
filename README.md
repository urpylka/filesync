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

### Что может сломаться

1. Переполнение лога программы
2. Не запуск программы из-за ошибки (например, db.json incorrect)
3. Хеш не совпадает
4. Нет места на сервере
5. Как помечать на сервере файлы, которые только передаются

### Один из алгоритмов

1. Начинаем качать в буффер сначала или с сохраненной позции
2. Спрашиваем сколько мы передали без ошибок
3. Смотрим, что у нас есть в буффере
4. Сбрасываем буффер (выбрасываем исключение в target.write()), если у нас позиция невходит в него

### Обрабатываемые ситуации

Данные случаи не должны приводить к остановке программы требующей участия человека, а также к перезаписи всего файла сначала

1. Выключение программы и RPi
2. Горячее изъятие и вставка флеш-карты
3. Выключение FTP

### TODO

* Нужнен гибкий конфигурационный файл с правилами, откуда, куда и что делать.
* Реализовать API для ведения прогресс-бара
* Если процесс непрерывный и последовательный, можно сделать одну числовую переменную, описывающую состояния файла. Например:
  * 0 - лог найден в source
  * 1 - лог скачен в local
  * 2 - лог выкачен в target
  * 3 - лог удален из source
  * 4 - лог выкачен не полностью

* Параллельная работа с FTP
* Проверка hash
* Reverse direction
* Надежность
* Last seek
* Progress bar (в буффер)

Filename
Down ↓ 1021444 / 2200000002 (70%) 1.4MB/s
Up   ↑  900023 / 2200000002 (62%) 700KB/s

Можно реализовать в методах _read() и _write(). Правда там был минус.

### Ошибки

1. Вот так можно сломать:

```bash
pi@raspberrypi:~ $ sudo mount /dev/disk/by-uuid/5C3D-DD9E /mnt
FUSE exfat 1.2.5
ERROR: 'Video_Tuner_WP_20151216_004_110904.mp4' points to invalid cluster 0.
```

2. Если качать с FTP на DISK и отключать диск, бывает так, что файл не сохраняет свою позицию. Мб нужно добавить в деструктор какой-нибудь внутренний Lock (ожидание чтобы операции download & upload завершились). Или что-то, что могло бы вызвать ошибку в их работе или прерывание.

### Файловая система exfat

For use exfat run:

```bash
sudo apt-get install exfat-fuse exfat-utils
```
