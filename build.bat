@echo off
setlocal

@REM rmdir /s /q build
@REM mkdir build
@REM cmake ..
cmake -G "MinGW Makefiles" -S . -B build
cmake --build build
