Carla settings:

- Running Carla:

.\CarlaUE4.exe /Game/Maps/Town/01 -carla-server -benchmark -fps=20 -windowed -ResX=400 -ResY=300

  
- Config:

The script ==PythonAPI/util/config.py== provides more configuration options and should be run when the server has been started:

  

Adding NPCs Non player Characters

  
  

Running Carla:

./CarlaUE4.exe -windowed -ResX=800 -ResY=600 -carla-server -quality-level=Low

  

./CarlaUE4.sh -windowed -ResX=800 -ResY=600 -carla-server -quality-level=Low

  

==Setting up the CARLA environment involves several steps. Here's a step-by-step guide to get you started:==

==Initial Setup for CARLA Environment==

**1. Prerequisites:**

- ==Operating System: CARLA supports Ubuntu 18.04 (or later) and Windows 10.==
- ==Graphics: A decent GPU. CARLA makes intensive use of the GPU.==
- ==Python: Python 3.7 or later.==

**2. Download CARLA:**

==Download the CARLA release that matches your operating system from the== ==official Github repository====.==

**3. Extract and Run CARLA:**

- ==For== ==Linux====:====￼====￼====bash====Copy code====￼====mkdir====carla== ==cd====carla tar -xvf CarlaUE4.tar.gz ./CarlaUE4.sh==
- ==For== ==Windows====: Just extract the zip file and run== ==CarlaUE4.exe====.==

==When run, you should see the simulator window open, showing a view of a city.==

**4. Python API:**

==Inside the extracted CARLA directory, there's a== ==PythonAPI== ==directory. This contains the necessary Python packages and examples to interact with the simulator via Python scripts.==

- ==Install the CARLA Python API in your Python environment:====￼====￼====bash====Copy code====￼====cd====PythonAPI/carla/dist pip install carla-*version*.tar.gz==

**5. Running Python Examples:**

==Go to the== ==PythonAPI/examples== ==directory in the CARLA folder. Here you can find various Python scripts showcasing different functionalities of the simulator.==

==For instance, to run the automatic control script:==

  

bashCopy code

python3 spawn_npc.py -n 100 # This will spawn 100 vehicles in the simulation.python3 automatic_control.py # This will start a vehicle that drives automatically.

**6. Additional Setup (Optional):**

- ==Town Maps: CARLA comes with several pre-built town maps, but you can also design custom maps using the Unreal Engine Editor.==
- ==Sensors: By default, CARLA provides a range of sensors like cameras, LiDAR, GNSS, etc. You can customize these sensors or even create new ones.==
- ==Weather Conditions: CARLA allows for changing weather conditions on-the-fly. You can simulate rain, fog, and different times of the day.==

**7. Documentation & Tutorials:**

==Make sure to go through the== ==official CARLA documentation====. It provides a comprehensive guide on every aspect of the simulator, from basic setups to advanced configurations and customizations.==

**8. Challenges & Leaderboard:**

==CARLA also organizes challenges where researchers and hobbyists can submit their autonomous driving solutions and see how they fare against others. Participating in these can provide a practical benchmark for your implementations.==

**Conclusion:**

==This setup should give you a basic CARLA environment up and running. From here, you can dive into more complex scenarios, integrate it with your decision-making framework, or even connect it to external tools and software. The possibilities are vast and can align well with the research direction you've chosen.==

  
