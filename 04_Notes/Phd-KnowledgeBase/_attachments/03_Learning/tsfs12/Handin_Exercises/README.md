# Python installation instructions

In the student labs, there is a pre-prepared virtual environment with all necessary packages installed. You activate the virtual environment as
```
% source /courses/tsfs12/env/bin/activate
```
If you install at home, we recommend that you create your own virtual environment for the hand-ins. First, open a terminal and ensure you have Python installed
```
% python --version  # Name of the binary may vary between installations
```
to verify that you have an up-to-date installation. A Python installation >=3.9 should be fine. All hand-ins are tested on Linux/Mac/Windows using Python 3.11.

Now, create a virtual environment as
```
% python -m venv env
% source env/bin/activate  # On Linux or Mac
% env\Scripts\activate  # On Windows
(env) % pip install -U pip  # Always a good idea to update the package installer
```
and then install all required packages with the command
```
(env) % pip install -r requirements.txt
``` 
You can find the file ```requirements.txt``` in the ```Handin_Exercises``` folder of this git repository.
