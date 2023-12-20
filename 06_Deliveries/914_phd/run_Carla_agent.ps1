echo off
P:\02_Tools\CARLA_0.9.14\WindowsNoEditor\CarlaUE4.exe -windowed -ResX=800 -ResY=600 -carla-server -quality-level=Low
conda activate py37
python ./belk/app.py
pause