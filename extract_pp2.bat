@echo off
set JDKBIN=C:\Users\Public\wpilib\2026\jdk\bin
set OUT=pp_src_extract
if exist %OUT% rmdir /s /q %OUT%
mkdir %OUT%
cd %OUT%
"%JDKBIN%\jar.exe" xf "C:\Users\techm\.gradle\caches\modules-2\files-2.1\com.pedropathing\core\3.0.1\ed369ea318ef7b76feb8de940060337d920cb6e1\core-3.0.1-sources.jar" > ..\extract_log.txt 2>&1
"%JDKBIN%\jar.exe" xf "C:\Users\techm\.gradle\caches\modules-2\files-2.1\com.pedropathing\revhub\3.0.1\4e2185db7e1ecef01cc0a54de844644828e13741\revhub-3.0.1-sources.jar" >> ..\extract_log.txt 2>&1
cd ..
dir /s /b %OUT%\*.java > pp_files.txt 2>&1
