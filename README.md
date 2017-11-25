# ros_px4logs
ROS package (by pubsub technology) for download & upload PX4 logs to logs.px4.io
___

For use this script bundle your PixHawk must connect to Companation Computer with USB (NOT UART!)
___

## Примечание:

Нельзя запрашивать несколько mavftp-команд параллельно, будет ошибка: "FTP: Busy". По этой причине нельзя создавать несколько downloader'ов, а также по этой причине введена mavftp_lock между downloader'ом и finder'ом. Аналогичным образом работает QGC.

## Возможные улучшения:

* Найти ошибку в crc32
* Понять нужны ли join()
* Загрузка чанками requests https://stackoverflow.com/questions/13909900/progress-of-python-requests-post
