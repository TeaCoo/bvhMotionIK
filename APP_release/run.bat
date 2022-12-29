@echo off
set positionFilePath=./position/positions_6.npy
set bvhFilePath=./bvh/Tue_01_p_06_IK.bvh
set outputFilePath=./output/Tue_01_p_06.bvh

for %%I in (%positionFilePath%) do set positionFileName=%%~nI

python npy2txt.py %positionFilePath%

set txtFileName=./position/%positionFileName%.txt

IK_processer.exe %txtFileName% %bvhFilePath% %outputFilePath%
