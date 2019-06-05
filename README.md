# FileSync

This software allows to sync a group of files between devices w difficult access:

* Pixracer (mavros ftp)
* FlirDuo (external flash drive)
* website (POST requests, developed for logs.px4.io)
* FTP server

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
6. Нужно добавить автоматическое создание бекапа бд

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

* Если процесс непрерывный и последовательный, можно сделать одну числовую переменную, описывающую состояния файла. Например:
  * 0 - лог найден в source
  * 1 - лог скачен в local
  * 2 - лог выкачен в target
  * 3 - лог удален из source
  * 4 - лог выкачен не полностью

* Reverse direction
* Надежность

### Ошибки

1. Вот так можно сломать:

```bash
pi@raspberrypi:~ $ sudo mount /dev/disk/by-uuid/5C3D-DD9E /mnt
FUSE exfat 1.2.5
ERROR: 'Video_Tuner_WP_20151216_004_110904.mp4' points to invalid cluster 0.
```

2. Если качать с FTP на DISK и отключать диск, бывает так, что файл не сохраняет свою позицию. Мб нужно добавить в деструктор какой-нибудь внутренний Lock (ожидание чтобы операции download & upload завершились). Или что-то, что могло бы вызвать ошибку в их работе или прерывание.


ротация логов
мб конфиг и бд где то в другом месте держать?


/mnt/20190403_122014/20190403_122537_816_R.jpg

20190403_122533_785_R.jpg.part

"target_path": "", "downloaded": true, "source_path": "/20190403_122014/20190403_122537_816_R.jpg", "uploaded": false, "dropped": false, "source_size": 196048


Apr 04 08:14:11 raspberrypi manager.py[314]: 2019-04-04 08:14:11,213 - ERROR - test-1@192.168.20.186: Renaming was interrupted: 550 No such file or directory.
Apr 04 08:14:11 raspberrypi manager.py[314]: 2019-04-04 08:14:11,984 - INFO - D1C6-0146: Started w 0
Apr 04 08:14:11 raspberrypi manager.py[314]: 2019-04-04 08:14:11,987 - ERROR - D1C6-0146: Downloading was interrupted: [Errno 2] No such file or directory: '/mnt/20190403_122014/20190403_122537_816_R.jpg'




[I 2019-04-04 15:07:33] ::ffff:192.168.20.172:55558-[test-1] CWD /Users/smirart/github/filesync/temp 250
[I 2019-04-04 15:07:33] ::ffff:192.168.20.172:55558-[test-1] RNFR /Users/smirart/github/filesync/temp/20190403_160336_487_R.jpg.part 550 'No such file or directory.'
[I 2019-04-04 15:07:34] ::ffff:192.168.20.172:55558-[test-1] CWD /Users/smirart/github/filesync/temp 250
[I 2019-04-04 15:07:34] ::ffff:192.168.20.172:55558-[test-1] RNFR /Users/smirart/github/filesync/temp/20190403_160336_487_R.jpg.part 550 'No such file or directory.'




drone@drone:~/Desktop/filesync$ sudo ./src/ftp_server.py 
Traceback (most recent call last):
  File "./src/ftp_server.py", line 7, in <module>
    from pyftpdlib.authorizers import DummyAuthorizer
ImportError: No module named pyftpdlib.authorizers


Добавить в логи разделение \t


мб разделить логи программы на 


если не скаченный файл уже удален нужно посмотреть его на удаленном сервере

https://tunnelblick.net


Filesync Synchronizer

1. путь + hesh + мб mountpoint
2. Выявление поддеревьев в деревьях
3. Нужно выделить максимально большое поддерево
4. мб нужно чтобы прога строила большой граф со всеми связями
5. Мб первоначально (для ускорения) строить деревья по hesh

Что такое хеш функция
https://www.chaynikam.info/chto-takoe-hash-fayla-i-kak-ego-uznat.html
Коллизия в хешах
https://habrahabr.ru/post/113127/
Примеры на python
http://qaru.site/questions/32672/get-md5-hash-of-big-files-in-python

import hashlib
def checksum_md5(filename):
    md5 = hashlib.md5()
    with open(filename
,'rb') as f: 
        for chunk in iter(lambda: f.read(128 * md5.block_size), b''):
            md5.update(chunk)
    return md5.hexdigest()

print checksum_md5("Downloads/Клубника.rar")

Обход директорий
https://www.severcart.org/blog/all/list_direcory_content_with_python/

https://pypi.org/project/webdavclient/
Добавить функционал выбора (удалять после перемещения и тд как в freefilesync)

сделать чтобы DISK работали на macos

- [ ] Логи px4 filesync
- [ ] Ротация логов filesync
- [ ] Csv для бд filesync
- [ ] Ротация логов filesync
- [ ] Что если при переименовании такой файл уже сцществует?
- [ ] Нужно ли качать если файл уже переименован и есть

что если файл filesync сможет забирать еще и почту???

https://tproger.ru/translations/python-gui-pyqt/



https://pythonworld.ru/gui/pyqt5-firstprograms.html
https://pythonworld.ru/gui/pyqt5-menustoolbars.html
https://pythonworld.ru/gui/pyqt5-layout.html
https://pythonworld.ru/gui/pyqt5-eventssignals.html
https://pythonworld.ru/gui/pyqt5-dialogs.html
https://pythonworld.ru/gui/pyqt5-widgets.html
https://pythonworld.ru/gui/pyqt5-widgets2.html
https://pythonworld.ru/gui/pyqt5-dragdrop.html
https://pythonworld.ru/gui/pyqt5-painting.html
https://pythonworld.ru/gui/pyqt5-customwidgets.html
https://pythonworld.ru/gui/pyqt5-tetris.html



import my_webdav.urn as ur

path = "/dasfa/пывп/вф"
dir = False

u = ur.Urn(path, dir)

print(u)

print(u.filename())



import my_webdav.urn as ur

path = "/dasfa/пывп/вф"
dir = True

u = ur.Urn(path, dir)

print(u)

print(u.filename())
