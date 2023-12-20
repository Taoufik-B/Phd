Setting up the steering angle \delta = tan^-1 (2Lsin alpha / KddVf)

  

Pure pursuit Algorithm

==For each instant in time:==

- ==Compute the look ahead distance== ==ld as== ==l_d = np.clip(K_dd * speed, min_ld, max_ld)====. The function== ==np.clip== ==is documented== ==here====.== ==K_dd====,== ==min_ld====, and== ==max_ld== ==are parameters that you can tune.==
- ==Find the target point TP as the intersection of the desired path with a circle of radius== ==ld around the rear wheel.==
- ==Using the target point coordinates== ==(x_tp,y_tp)====, determine== ==α as== ==alpha=arctan2(y_tp,x_tp)==
- ==Use equation== ==(11)== ==to compute the pure pursuit front wheel angle== ==δ==
- **Act**==: Turn your steering wheel to set the front wheel angle to== 
  
> From <[https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html)>  
  

  

Pure pursuit algorithm implmented

  

Key takeway, use arctan2 to compute alpha angle.

  
  

[Carla Setup guide](https://d3c33hcgiwev3.cloudfront.net/IFfxQie8Eem9HA6xGGaRfg_20f6060027bc11e98ed3dfcfdba7c72b_CARLA-Setup-Guide-_Windows-x64_.pdf?Expires=1683590400&Signature=dGVEUdGIT8HmazHmrnmXnhrzHnnV5HGR3EpjioyfZL50HudJADypqWcWdi-mpvPJ9ALa9OM0EHwIGby6abQocdk9JiN0Awp04O9GPxfnoB0QDcU0taVjECGNbBhxWDL6BTN2VqTIyrrRHNPKUOpp~cNhAGUKyOtY1ZxfKjSUeB0_&Key-Pair-Id=APKAJLTNE6QMUY6HBC5A)

  
  
  

CarlaUE4.exe -windowed -carla-no-networking

  
  

**RaceTrack**:

CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-no-networking

  

With fps 20

CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-no-networking -benchmark -fps=20

==./CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=20==

  

./CarlaUE4.exe /Game/Maps/RaceTrack -windowed -ResX=800 -ResY=600 -carla-server -benchmark -fps=20

  

conda activate py36

python module_7.py

  
  
> From <[https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html)>