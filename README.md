# ros_px4logs

ROS package (by pubsub technology) for download from PX4 & upload logs to logs.px4.io

## Примечание

1. For use this script bundle your PixHawk must connect to Companation Computer with USB (NOT UART!)
2. Нельзя запрашивать несколько mavftp-команд параллельно, будет ошибка: "FTP: Busy". По этой причине нельзя создавать несколько downloader'ов, а также по этой причине введена mavftp_lock между downloader'ом и finder'ом. Аналогичным образом работает QGC.

## TODO

* Найти ошибку в crc32 для проверки файлов по контрольной сумме
* Понять нужны ли join() для thread'ов
* Загрузка чанками requests https://stackoverflow.com/questions/13909900/progress-of-python-requests-post
* Замена json_database на http://tinydb.readthedocs.io/en/latest/
* Добавление сервисов для работы с внутренней базой данных
* Брать описание для логов с /etc/coex.conf
* Поэксперементировать с относительнымы путями для директории логов
* Ускорить загрузку логов https://github.com/mavlink/mavros/issues/874
