@echo off
cd /d "%~dp0"
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /std:c++17 /O2 /EHsc /utf-8 /Fe:local_search_test.exe csv_loader.cpp graph.cpp evaluate.cpp ga.cpp tests\local_search_test.cpp
if errorlevel 1 exit /b 1
"%~dp0local_search_test.exe"
