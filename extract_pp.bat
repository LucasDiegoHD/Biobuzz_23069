@echo off
set JDKBIN=C:\Users\Public\wpilib\2026\jdk\bin
set SRC=C:\Users\techm\.gradle\caches\modules-2\files-2.1\com.pedropathing\core\3.0.0\d4d3d1d3dcbd92bc769d36f0b9cef77a520b9fe5\core-3.0.0-sources.jar
set OUT=pp_src_extract
if exist %OUT% rmdir /s /q %OUT%
mkdir %OUT%
cd %OUT%
"%JDKBIN%\jar.exe" xf "%SRC%" > ..\extract_log.txt 2>&1
cd ..
