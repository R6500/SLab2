# SLab2

New version of the **SLab** system

The SLab system is a low cost electronics Lab aimed to teaching electronics

It uses a cheap demostration board to interface with a circuit under test

User interface is built around **Jupyter Notebooks** that contain **Python** Code

In order to use the software you need to have a working Python 3 system with Jupyter installed

Jupyter shall start on the *"Jupyter"* folder

Before starting Jupyter, the **PHYTHONPATH** variable should point to the *"Code"* folder

The *"Anaconda_SLab_Jupyter.bat"* in the *"Scripts"* folder provides one example for Windows that works for an **Anaconda** installation:

`SET PATH=%PATH%;\Anaconda_Win64  
SET PATH=%PATH%;\Anaconda_Win64\Scripts  
SET PATH=%PATH%;\Anaconda_Win64\Library\bin;  
set PYTHONPATH=\SLab\Code  
cd \SLab\Jupyter  
jupyter notebook`  

