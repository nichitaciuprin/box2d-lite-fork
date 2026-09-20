@echo off
setlocal

echo y | gdb -q -ex=run -ex=backtrace -ex=quit --args %cd%\build\tests\Test1.exe
