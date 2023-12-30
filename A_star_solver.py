'''
=================================================================================
                            RUBIK CUBE A* SOLVER
=================================================================================
- Author: Daniel Sotelo Aguirre
- Github: https://github.com/danisotelo
- E-mail: d.sotelo@alumnos.upm.es
- Subject: Guidance and Navigation of Robots
- Date: December 29, 2023
=================================================================================
This code solves the Rubik Cube by using the A* algorithm and a heuristics
based on the required number of movements to correctly position and orient all
the cubies. The following resources have been employed:

- PyRubikSim by Miguel Hernando: https://github.com/mhernando/pyRubikSim
- Magic Cube by David W. Hogg and Jake Vanderplas:
  https://github.com/davidwhogg/MagicCube
=================================================================================
'''

# Import libraries
import numpy as np
import threading
import time
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from PyQt5 import QtWidgets, QtCore

import rub_cube
import cube_interactive
import render_images


# In this table it has been codified where in the tuple each sticker of each 
# cubie is located. There are a total of 8 corners and 12 edges (corners have
# 3 stickers and edges have 2 stickers). This way it is located in the tuple
# where the stickers belonging to the same cubie are positioned. The codification
# starts with the corners: 0, 1, ..., 7 and then continues with the edges until
# 19. Numbers 20 to 25 codify the center-face stickers.
cube_map = (((0,  8, 1), ( 9, 20, 10), (2, 11, 3)),
            ((1, 12, 4), (10, 21, 13), (3, 14, 5)),
            ((4, 15, 6), (13, 22, 16), (5, 17, 7)),
            ((6, 18, 0), (16, 23,  9), (7, 19, 2)),
            ((0, 18, 6), ( 8, 24, 15), (1, 12, 4)),
            ((3, 14, 5), (11, 25, 17), (2, 19, 7)))

# Dictionary to know which sticker belong to each cubie in the solution. The 
# key of the dictionnary is the cubie and the list contains the colors of the
# stickers of that cubie.
dict_cubies = { 0: [0, 3, 4],  1: [0, 1, 4],  2: [0, 3, 5],  3: [0, 1, 5],
                4: [1, 2, 4],  5: [1, 2, 5],  6: [2, 3, 4],  7: [2, 3, 5],
                8:    [0, 4],  9:    [0, 3], 10:    [0, 1], 11:    [0, 5],
               12:    [1, 4], 13:    [1, 2], 14:    [1, 5], 15:    [2, 4],
               16:    [2, 3], 17:    [2, 5], 18:    [3, 4], 19:    [3, 5]}

# Graph node
class AStarNode:
    def __init__(self, state, visited, parent, g, h, rotation):
        self.state = state
        self.visited = visited
        self.parent = parent
        self.rotation = rotation # To keep track of the rotation that was made to arrive to that node
        self.g = g # Cost to reach this node from the initial node
        self.h = h # Heuristic estimation to final node
        self.f = self.g + self.h # Total estimated cost
    
    # Methods to compare node costs    
    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return self.state == other.state
    
    def __gt__(self, other):
        return self.f > other.f
    
    # Method to calculate the costs of a node
    def calculate_cost(self, parent):
        if parent is not None:
            g = parent.g + 1
        else:
            g = 1
        
        f = g + self.h
        
        return g, f
   
    # Method to update the costs of a node
    def update_cost(self):
        self.g, self.f = self.calculate_cost(self.parent)
        
    # Method for updating parent to the one with least cost
    def update_parent(self, cheaper_parent):
        _, f = self.calculate_cost(cheaper_parent)
        
        if  f < self.f:
            self.parent = cheaper_parent
            self.update_cost()
    
# A* solver
class AStarSolver:
    
    # The divisor has been set as initialization variable because several divisors were tried
    # and 5 was the one for which the best results were observed
    def __init__(self, cube = rub_cube.RubCube(3), Q = None, visited_nodes = None, divisor = 5):
        self.cube = cube
        self.Q = Q if Q is not None else []  
        self.visited_nodes = visited_nodes if visited_nodes is not None else []
        self.divisor = divisor
    
    # Function to check if cubie is correctly positioned
    # The stickers of that cubie are located in the same cubie (can be in different orientation)
    def correct_position(self, list_of_pairs, cubie):
        boolean = True
        
        if cubie in range(0, 20):
            for couple in list_of_pairs:
                if couple[0] == cubie:
                    if couple[1] not in dict_cubies[cubie]:
                        boolean = False
        
        return boolean
    
    # Function to check if cubie is correctly oriented
    # At least one of the stickers of the cubie is located on its face
    def correct_orientation(self, list_of_pairs, cubie):
        boolean = False
        
        if cubie in range(0, 20):
            i = 0
            for couple in list_of_pairs:
                if couple[0] == cubie: # For the desired cubie
                    if couple[1] == dict_cubies[cubie][i]: # If the state of that sticker is the same as the color of the corresponding face
                        boolean = True
                        return boolean
                    else:
                        i += 1 # Jump to next face
                        
            return boolean
    
    # Function saying the number of moves required to correctly
    # position and orient a cubie
    def moves_for_correct_cubie(self, list_of_pairs, cubie):
        bool_pos = self.correct_position(list_of_pairs, cubie)
        bool_orient = self.correct_orientation(list_of_pairs, cubie)
        
        # If it is a corner
        if cubie in range(0, 8):
            if bool_orient == True:
                if bool_pos == True:
                    return 0
                else:
                    return 1
            else:
                return 2
        # If it is an edge
        if cubie in range(8, 20):
            if bool_pos == True:
                if bool_orient == True:
                    return 0
                else:
                    return 3
            else:
                if bool_orient == True:
                    return 1
                else:
                    return 2
       
    # HEURISTICS: Number of moves required for positioning and orientation of all the cubies
    def heuristic(self, state, divisor):
        sum_moves = 0
        
        # Creating a list of lists of two numbers
        list_of_pairs = [[pair[0], pair[1]] for face in zip(cube_map, state) for row in zip(face[0], face[1]) for pair in zip(row[0], row[1])]
        
        # Check the number of moves for each cubie and add it to the total sum
        for i in range(20):
            sum_moves += self.moves_for_correct_cubie(list_of_pairs, i)
            
        # Divide by a number to make the heuristic admissible and have the best results              
        return sum_moves / divisor         
    
    # Method to check if a node has already been visited
    def is_visited(self, node):
        return node in self.visited_nodes
    
    # Method to get the best node from the queue
    def get_best_node(self):
        if not self.Q:
            return None # Return None if the queue is empty
        
        best_node = min(self.Q, key = lambda node: node.f) # Find the node with the smallest f value
        self.Q.remove(best_node) # Remove the best node from the queue
        return best_node
    
    def get_next_states(self, parent):
        next_states = []
        axes = ['x', 'y', 'z'] # Possible axes of rotation
        layers = [0, 2]
        senses = [-1, 1, 2] # Possible senses of rotation
        for axis in axes:
            for layer in layers: # Possible layers of rotation
                for sense in senses:
                    aux_cube = rub_cube.RubCube(3)
                    aux_cube.set_State(self.cube.get_State()) # Create auxiliary cube with the same state
                    aux_cube.rotate_90(axis, layer, sense)
                    aux_state = aux_cube.get_State() # Rotate and update state of auxiliary cube
                    next_state = AStarNode(aux_state, False, parent, parent.g + 1, self.heuristic(aux_state, self.divisor), [(axis, layer, sense)]) # Create node for new state
                    next_states.append(next_state)
        return next_states
    
    def solve(self):
        # Initialize the A* solver
        rotations = []
        initial_node = AStarNode(self.cube.get_State(), False, None, 0, self.heuristic(self.cube.get_State(), self.divisor), [('x', 0, 0)])
        self.visited_nodes.append(initial_node)
        self.Q.append(initial_node)
        ended = False
        final_node = AStarNode(rub_cube.RubCube(3).get_State(), False, None, 0, 0, None)
        print("                                                 Exploring nodes...                                              ")
        
        # Main loop
        while not ended and len(self.Q) > 0:
            x = self.get_best_node()
            self.cube.set_State(x.state)
            
            if x.state == final_node.state:
                print("--------------------------------------------------------------------------------------------------------------")
                print("                                          THE CUBE HAS BEEN SOLVED! :D                                           ")
                print("--------------------------------------------------------------------------------------------------------------")
                ended = True
                # Once the solution is found we go back through the parents until reaching the initial node
                parent = x.parent
                while parent is not None:
                    rotations.insert(0, x) # Insert the solution nodes in reverse order
                    x = parent
                    parent = x.parent
            
            else:
                U = self.get_next_states(x)
                for u in U:
                    #print(u.state)
                    if not self.is_visited(u):
                        self.Q.append(u)
                        self.visited_nodes.append(u)
                    else:
                        u.update_parent(x)
        
        return rotations
    
def get_initial_random_movements():
    # Rotate the cube randomly X movements
    while True:
        try:
            X = int(input("Please, enter the number of initial random movements: "))
            if X >= 1:
                return X
            else:
                print("The number must be greater than or equal to 1. Please try again.")
        
        except ValueError:
            print("Invalid input. Please enter an integer number.")
            
def get_bool_render_img():
    # Get a boolean from the user for image rendering preferences
    while True:
        try:
            X = int(input("Do you want to render the moves images? (1 / 0): "))
            if X == 1 or X == 0:
                return X
            else:
                print("You must introduce 1 (True) or 0 (False). Please try again.")
        
        except ValueError:
            print("Invalid input. Please enter 1 or 0.")
            
def get_bool_animation():
    # Get a boolean from the user for animation preferences
    while True:
        try:
            X = int(input("Do you want to see the animation of the cube? (1 / 0): "))
            if X == 1:
                print("\n- Note: You can access the generated images in the folder images/")
                return X
            elif X == 0:
                return X
            else:
                print("You must introduce 1 (True) or 0 (False). Please try again.")
        
        except ValueError:
            print("Invalid input. Please enter 1 or 0.")
            
def on_close(event, bool_anim, moves_solve):
    '''
    This function will be called when the animation window is closed.
    It continues with the solving part of the program.
    '''
    solver = AStarSolver(cube)
    rotations = solver.solve()
    
    print("\nA total of", len(solver.visited_nodes), "nodes were explored!")
    print("\nNumber of rotations for solving: ", len(rotations))
    for i in range(len(rotations)):
        move = rotations[i].rotation
        print(" - Rotation ", i + 1, ": ", move)
        # Need to encode the rotations to be understood by cube_interactive.py
        if bool_anim:
            if move[0][0] == "x":
                moves_solve.append(("F", -move[0][2], move[0][1]))
            elif move[0][0] == "y":
                moves_solve.append(("R", -move[0][2], move[0][1]))
            elif move[0][0] == "z":
                moves_solve.append(("U", -move[0][2], move[0][1]))
    
    if bool_anim:
        fig = plt.figure(figsize = (5, 5))
        interactive_cube = cube_interactive.InteractiveCube(cube_anim, fig = fig)
        fig.add_axes(interactive_cube)

        # Thread used to update the figure each 2 seconds
        update_thread = threading.Thread(target = cube_interactive.update_cube, args = (cube_anim, interactive_cube, 2, moves_solve))
        update_thread.start()
        plt.show()
        print("\nHere you can see the steps to solve the cube. You can rotate the cube by moving the mouse!")
        print("Close the animation window to end the program!")
    
# CODE FOR MAIN PROGRAM
if __name__ == "__main__":
    
    # NORMAL USE
    
    # Create the Rubik Cube
    cube = rub_cube.RubCube(3)
    
    print("\n**************************************************************************************************************\n",
          "                                              RUBIK CUBE A* SOLVER                                              ",
          "\n**************************************************************************************************************\n")
    
    # Take preference from the user
    X = get_initial_random_movements()
    bool_img = get_bool_render_img()
    bool_anim = get_bool_animation()
    print("\n**************************************************************************************************************")
    
    # Save initial images
    if bool_img:
        cube_img = render_images.Cube(3, whiteplastic = False)
        cube_img.render(flat = True).savefig("images/flat/position_flat_0.png", dpi=865 / cube_img.N)
        cube_img.render(views = True).savefig("images/all/position_0.png", dpi=865 / cube_img.N)
        cube_img.render(flat = False, views = True).savefig("images/views/position_views_0.png", dpi=865 / cube_img.N)
        # Close the plots to prevent them from displaying
        plt.close('all')
    
    # Set up the animation variables    
    if bool_anim:
        moves_anim = []
        moves_solve = []
        cube_anim = cube_interactive.Cube(3)
    
    # Move the cube randomly X+1 times and print the results of the random rotations
    print("\nInitial random movements: ")
    for x in range(X):
        moves = cube.randomMoves(1)
        print(" - Rotation ", x + 1, ": ", moves)
        
        # Need to encode the rotations to be understood by render_images.py
        if bool_img:
            if moves[0][0] == "x":
                cube_img.move("F", moves[0][1], -moves[0][2])
            elif moves[0][0] == "y":
                cube_img.move("R", moves[0][1], -moves[0][2])
            elif moves[0][0] == "z":
                cube_img.move("U", moves[0][1], -moves[0][2])
            
            # Render the images for each rotation (flat, views and all)        
            cube_img.render(flat = True).savefig("images/flat/position_flat_%01d.png" % (x + 1), dpi=865 / cube_img.N)
            cube_img.render(views = True).savefig("images/all/position_%01d.png" % (x + 1), dpi=865 / cube_img.N)
            cube_img.render(flat = False, views = True).savefig("images/views/position_views_%01d.png" % (x + 1), dpi=865 / cube_img.N)
            plt.close("all")
        
        # Need to encode the rotatinos to be understood by cube_interactive.py
        if bool_anim:
            if moves[0][0] == "x":
                moves_anim.append(("F", -moves[0][2], moves[0][1]))
            elif moves[0][0] == "y":
                moves_anim.append(("R", -moves[0][2], moves[0][1]))
            elif moves[0][0] == "z":
                moves_anim.append(("U", -moves[0][2], moves[0][1]))
    
    # Plot animation figure of the cube rotations            
    if bool_anim:
        fig = plt.figure(figsize = (5, 5))
        interactive_cube = cube_interactive.InteractiveCube(cube_anim, fig = fig)
        fig.add_axes(interactive_cube)
        
        # Retrieve the window and set it to be always on top
        window = plt.get_current_fig_manager().window
        window.setWindowFlags(window.windowFlags() | QtCore.Qt.WindowStaysOnTopHint)

        update_thread = threading.Thread(target = cube_interactive.update_cube, args = (cube_anim, interactive_cube, 1, moves_anim))
        update_thread.start()
        
        print("\nHere you can see the initial state of the cube to solve. You can rotate the cube by moving the mouse!")
        print("Close the animation window to start solving the cube!")
        print("\n**************************************************************************************************************")
        
        # Connect the close event of the figure to the on_close function
        fig.canvas.mpl_connect('close_event', lambda event: on_close(event, bool_anim, moves_solve))
        
        plt.show()
    
    # If no animation is desired    
    else:
        print("\n**************************************************************************************************************")
        solver = AStarSolver(cube)
        rotations = solver.solve()
        
        print("\nA total of", len(solver.visited_nodes), "nodes were explored!")
        print("\nNumber of rotations for solving: ", len(rotations))
        for i in range(len(rotations)):
            move = rotations[i].rotation
            print(" - Rotation ", i + 1, ": ", move)
    print("\n**************************************************************************************************************")

    '''
    # CODE FOR DETERMINING THE OPTIMAL HEURISTIC DIVISOR
    
    # Create the Rubik Cube
    cube = rub_cube.RubCube(3)
    avg_times = []
    
    X = 5 # Number of moves
    
    for divisor in range(4, 9):
    
        times = []
        
        for i in range(10):
            print("Initial random movements: ")
            for x in range(X):
                move = cube.randomMoves(1)
                
            # Record the start time
            start_time = time.time()
        
            solver = AStarSolver(cube = cube, divisor = divisor)
            rotations = solver.solve()
            
            # Record the end time
            end_time = time.time() 
            
            # Calculate the elapsed time
            elapsed_time = end_time - start_time
            
            times.append(elapsed_time)
            
            cube.reset()
        
        avg_time = sum(times)/len(times)
        avg_times.append(avg_time)

    print(avg_times)
    '''
    
    '''
    # CODE FOR DETERMINING THE AVERAGE COMPUTING TIME FOR DIFFERENT NUMBER OF RANDOM MOVES (DIVISOR = 5)
    
    # Create the Rubik Cube
    cube = rub_cube.RubCube(3)
    divisor = 1
    avg_times = []
    
    for X in range(1, 8):
    
        times = []
        
        for i in range(10):
            print("Initial random movements: ")
            for x in range(X):
                move = cube.randomMoves(1)
                
            # Record the start time
            start_time = time.time()
        
            solver = AStarSolver(cube = cube, divisor = divisor)
            rotations = solver.solve()
            
            # Record the end time
            end_time = time.time() 
            
            # Calculate the elapsed time
            elapsed_time = end_time - start_time
            
            times.append(elapsed_time)
            
            cube.reset()
        
        avg_time = sum(times)/len(times)
        avg_times.append(avg_time)

    print(avg_times)
    '''
    
    