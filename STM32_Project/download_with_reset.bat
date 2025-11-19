@echo off
echo ========================================
echo STM32F1 下载脚本 (复位模式)
echo ========================================
echo.
echo 请按照以下步骤操作:
echo 1. 按住 STM32F1 板子上的 RESET 按钮
echo 2. 按任意键开始连接...
pause >nul
echo.
echo 正在连接...请在看到 "Interface ready" 后立即松开 RESET 按钮!
echo.

D:\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe -s D:\DevEnv\openocd-v0.12.0-i686-w64-mingw32\share\openocd\scripts -f D:\DevEnv\stm32f1_dap.cfg -c "tcl_port disabled" -c "gdb_port disabled" -c "program \"D:/Linux学习/STM32_Class_Project/STM32_Class_Project/build/Debug/STM32_Class_Project.elf\" verify reset exit"

echo.
echo ========================================
echo 下载完成!
echo ========================================
pause
