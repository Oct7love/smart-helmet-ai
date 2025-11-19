@echo off
echo ========================================
echo STM32F1 全片擦除脚本
echo ========================================
echo.
echo 警告: 此操作会擦除芯片中的所有数据!
echo.
echo 请按住 RESET 按钮,然后按任意键继续...
pause >nul
echo.
echo 正在擦除...请在看到 "Interface ready" 后立即松开 RESET!
echo.

D:\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe -s D:\DevEnv\openocd-v0.12.0-i686-w64-mingw32\share\openocd\scripts -f D:\DevEnv\stm32f1_dap.cfg -c "init" -c "reset halt" -c "stm32f1x mass_erase 0" -c "reset run" -c "shutdown"

echo.
echo ========================================
echo 擦除完成!
echo ========================================
pause
