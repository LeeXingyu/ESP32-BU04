@echo off
setlocal
pushd "%~dp0"
python test_leg.py --auto-dispatch %*
echo.
echo Script finished.
pause
popd
endlocal
