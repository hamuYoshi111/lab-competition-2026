@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /std:c++17 /O2 /EHsc /utf-8 /Fe:orienteering.exe csv_loader.cpp graph.cpp evaluate.cpp ga.cpp main.cpp
