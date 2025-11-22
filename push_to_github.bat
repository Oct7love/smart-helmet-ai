@echo off
echo ========================================
echo 推送项目到GitHub
echo ========================================
echo.

cd /d "%~dp0"

echo [1/3] 添加远程仓库...
git remote add origin https://github.com/Oct7love/smart-helmet-ai.git

echo.
echo [2/3] 设置默认分支为main...
git branch -M main

echo.
echo [3/3] 推送到GitHub...
git push -u origin main

echo.
echo ========================================
echo 推送完成！
echo 访问: https://github.com/Oct7love/smart-helmet-ai
echo ========================================
pause
