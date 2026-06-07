@echo off
setlocal
set "PROJECT_ROOT=%~dp0.."
powershell -ExecutionPolicy Bypass -File "%PROJECT_ROOT%\scripts\build_orbslam3_windows.ps1" %*
