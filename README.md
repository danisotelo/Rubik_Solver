# Rubik's Cube Solver

The following is a code that solves the Rubik 3x3x3 Cube by using A* algorithm. The code is an implementation using the simulator to test discrete planning techniques of Miguel Hernando in https://github.com/mhernando/pyRubikSim and the visualization utils from David Hogg in https://github.com/davidwhogg/MagicCube. 

The program enables to solve a Rubik's Cube given a certain amount of random initial movements. The program offers the possibility to render images of the different states of disassembly of the cube, as well as a 3D interactive animation of the disassembly and assembly of the Rubik Cube. The state codification of the cube can be checked at the repository of Miguel Hernando. The heuristics for the A* algorithm is based on the required number of movements to correctly position and orient all the cubies.

<p align="center">
  <img src="https://github.com/danisotelo/monza_simulator/blob/master/img/solver.png" alt="Image Description" width="100%">
</p>

## Getting Started
### Cloning the Repository
To clone the repository and start using the **Rubik Solver**, follow these steps:
1. Open a terminal or command prompt.
2. Navigate to the directory where you want to clone the repository.
3. Run the following command:
```
git clone https://github.com/danisotelo/Rubik_Solver.git
```
### Installing Dependencies
Before running the program, you need to install the required dependencies. To install them run:
```
pip install numpy
pip install matplotlib
pip install PyQt5
```

## Running the Program
After cloning the repository and installing the required dependencies, navigate to the directory containing the **Rubik Solver** code and start the program.
1. Run in the terminal the main program: ``` python A_star_solver.py ```.
2. Select the number of initial random movements in the terminal.
3. Select if you want to render images into the ```images/``` folder with the steps.
4. Select if you want to display an animation of the disassembly and assembly of the cube.
5. See the movements in the console. When the animation is finished close it to solve the cube.
6. After exploring the nodes, the program will tell when it has solved the Rubik. Information about the solution can be found in the terminal. Additionally, you can also see the animation of the cube solution steps.
7. Close the animation window to finish the program.
   
### Note
The algorithm performs well and has been tested between 1 and 7 initial movements. If you want maximum performance disable the image rendering and animation options.
