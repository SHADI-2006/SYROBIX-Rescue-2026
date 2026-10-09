@echo off
rem Runs the project-local PlatformIO (venv). The package dir is set by core_dir in platformio.ini.
"%~dp0.venv\Scripts\pio.exe" %*
