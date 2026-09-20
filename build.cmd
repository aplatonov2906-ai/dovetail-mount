@echo off
cd /d "%~dp0"
python mount.py
python render.py
pause
