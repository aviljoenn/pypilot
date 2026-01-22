cd C:\gitsrc\inno-pilot\inno-remote\firmware\inno_remote
idf.py build
idf.py -p COM9 -b 921600 app-flash
idf.py -p COM9 monitor --no-reset