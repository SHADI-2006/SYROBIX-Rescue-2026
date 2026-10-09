@echo off
rem Serves this folder on http://localhost:8765 (Web Serial needs http://localhost or https).
rem Uses the project's own Python from ..\..\.venv. Close this window to stop the server.
cd /d "%~dp0"
start "" "http://localhost:8765/"
"%~dp0..\..\.venv\Scripts\python.exe" -m http.server 8765
