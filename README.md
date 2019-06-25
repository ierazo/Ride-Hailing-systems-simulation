# Ride-Hailing-systems-simulation

This repository contains files that allow for the simulation of ride-hailing systems, particularly considering 
four different behaviors for the drivers. 
1) A no movement scenario while waiting for passengers
2) A single movement where they decide where to go while waiting for passenger
3) A ''Nearest'' hotspot scenario, they go roaming to the clases hotspot, and a list of hotspots is given by the
ride-hailing company.
4) A ''Coordinated'' hotspot scenario, the driver just goes roaming to the destination the company tells him.

The Simulation.py file is where the simulation gets executed.
The Classes.py has all the different functions and classes of the simulation model.
from_to_generator.py is used to define the spatio-temporal behavior of drivers and passengers.
CreatingGraph.py is a file that has some functions used to test the code before running the case of study, but 
it is used too to compute the dictionaries used to speed up the simulation.

In fact, all the required files are available in this repository but ''SmallDict.pickle'' and "FullDict.pickle''.
We could not uplaod it because of their size (more than 800 MB), but running the CreatingGraph.py file after removing the # would generate you the two missing files.

In case of doubts don't doubt contacting us at erazoignacio@gmail.com
This was done as an undergradute thesis, so the pdf file with more details could be shared with you.