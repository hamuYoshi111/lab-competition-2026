@echo off
cd /d "%~dp0"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /std:c++17 /O2 /EHsc /utf-8 /Fe:efficiency_test.exe csv_loader.cpp graph.cpp evaluate.cpp ga.cpp tests\efficiency_test.cpp
if errorlevel 1 exit /b 1
"%~dp0efficiency_test.exe"
