@echo off
echo Starting demo server...
start /B python "%~dp0serve.py"
timeout /t 1 /nobreak >nul
start "" "http://localhost:8080/warehouse_amr_demo_test.html"
echo Server running at http://localhost:8080/warehouse_amr_demo_test.html
echo Close this window to stop the server.
pause
