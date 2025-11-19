@echo off
cd /d D:\linux_study\esp32\esp32_project\xiaozhi-esp32-clean
call D:\linux_study\esp32\Espressif\frameworks\esp-idf-v5.5.1\export.bat
idf.py set-target esp32s3
