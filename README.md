# MPC_MED
Model predictive control (MPC) of a momentum exchange device (MED)

## Contents
This repository contains the code corresponding with the thesis 'Towards model predictive control of a prosthetic leg actuated by a momentum exchange device' by Jesper Kreuk.
This includes code and models from 
- S. Song and H. Geyer, “A neural circuitry that emphasizes spinal feedback generates diverse behaviours of human locomotion,” The Journal of physiology, vol. 593, no. 16, pp. 3493–3511, 2015.
- H. Geyer and H. Herr, “A muscle-reflex model that encodes principles of legged mechanics produces human walking dynamics and muscle activities,” IEEE Transactions on neural systems and rehabilitation engineering, vol. 18, no. 3, pp. 263–273, 2010.
- N. Timmers, “Simulating gait with the 3r60 knee prosthesis and a control moment gyroscope,” 2020.
Their (modified) files are in the folders 'Song', 'Geyer' and 'Timmers', respectively.
 
The code has three main files
* CGBgreybox.m
* CGBcontrol.m
* NMScontrol.m

Files corresponding to the CGB model are in the folder 'CGB'
Files corresponding to the NMS model are in the folder 'NMS'
Files and images are in folder 'Results'

## Run the code
MATLAB 2019a is used to run the code, later distributions no longer support the first generation Simscape library
The following toolboxes in MATLAB are used:
* Simulink
* Simmechanics
* Simscape
* Optimization toolbox 
* Identification toolbox
* Symbolic toolbox

For solving the MIQP optimization the following toolbox is used:
* Gurobi optimizer (911)
 
Any of the three aforementioned main files can be run after these are installed

## License
The code is available for Academic or Non-Profit Organization Noncommercial research use only.
