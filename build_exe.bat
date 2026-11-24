@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo [1/3] 检查 PyInstaller ...
python -m PyInstaller --version >nul 2>&1
if errorlevel 1 (
    echo     未安装，正在安装 ...
    python -m pip install pyinstaller
    if errorlevel 1 goto fail
)

echo [2/3] 清理旧产物 ...
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist

echo [3/3] 打包（单文件、无控制台窗口）...
python -m PyInstaller --noconfirm --clean --onefile --windowed --name uart_tool uart_tool.py
if errorlevel 1 goto fail

echo.
echo 打包完成：dist\uart_tool.exe
echo 提示：exe 首次启动会解压到临时目录，约 1-3 秒属于正常现象。
pause
exit /b 0

:fail
echo.
echo 打包失败，请查看上面的错误输出。
pause
exit /b 1
