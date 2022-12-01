from path_generators.path_generators.triangulation import TraingPath
from path_generators.path_generators.skidpad_path import Skidpad
from path_generators.path_generators.path_generator_base import CarPos, Cone

import numpy as np
import matplotlib.pyplot as plt

import sys

sys.path.insert(0, '/Users/mario/Driverless/')

from id_track_utils.src.id_track_utils.track import Track

def plot_track(track, generated_path, car_pos):
    """Plot track

    Args:
        track (id_track_utils.src.id_track_utils.track.Track): track of the circuit
        generated_path (list(path)): ordered list of midpoints
        car_pos (path_generators.path_generators.path_generator_base.CarPos): Position of car
    """
    # Start path plotted in magenta
    plt.plot(generated_path[0][0], generated_path[0][1], 'm')
    plt.scatter(np.array(generated_path)[1:-3,0], np.array(generated_path)[1:-3,1])
    # End path plotted in black
    plt.plot(generated_path[-2][0], generated_path[-2][1], 'k')

    path = np.array(generated_path)

    cones_l = np.array(track.left_cones)
    cones_r = np.array(track.right_cones)
    small_orange = np.array(track.orange_cones)
    big_orange = np.array(track.big_orange_cones)

    plt.plot(path[:-1,0], path[:-1,1], c='g')

    plt.scatter(cones_l[:,0]-track.start_x, cones_l[:,1]-track.start_y, c='b')
    plt.scatter(cones_r[:,0]-track.start_x, cones_r[:,1]-track.start_y, c='yellow')
    plt.scatter(small_orange[:,0]-track.start_x, small_orange[:,1]-track.start_y, c='orange')
    plt.scatter(big_orange[:,0]-track.start_x, big_orange[:,1]-track.start_y, c='gold')

    # Car plotted in red
    plt.plot(car_pos.x, car_pos.y, 'ro')

    plt.show()

def skidpad_path_beg_centre(track):
    """Generate skidpad path from beginning to centre
    Args:
        track (id_track_utils.src.id_track_utils.track.Track): track of the circuit
    Returns:
        generated_path (list(path)): ordered list of midpoints
    """    

    skidpad = Skidpad()
    big_orange_cones = [Cone(x-track.start_x, y-track.start_y) for x, y in track.big_orange_cones]
    # print("big_orange_cones", big_orange_cones)

    small_orange_cones = [Cone(x-track.start_x, y-track.start_y) for x, y in track.orange_cones]
    # print("small_orange_cones", small_orange_cones)
    orange_cones = small_orange_cones + big_orange_cones

    left_cones, right_cones = skidpad.relabel_orange(orange_cones, n=8)
    triangulisation = TraingPath()
    generated_path = triangulisation.generate_path(left_cones, right_cones)
    
    return generated_path

def skidpad_path_left_circle(track):
    """Generate skidpad path from of left circle
    Args:
        track (id_track_utils.src.id_track_utils.track.Track): track of the circuit
    Returns:
        generated_path (list(path)): ordered list of midpoints
    """ 
    left = True
    skidpad = Skidpad()
    left_cones = [Cone(x-track.start_x, y-track.start_y) for x, y in track.left_cones]
    right_cones = [Cone(x-track.start_x, y-track.start_y) for x, y in track.right_cones]
    left_cones, right_cones = skidpad.recolour(left_cones, right_cones, left)
    triangulisation = TraingPath()
    generated_path = triangulisation.generate_path(left_cones, right_cones)
    
    return generated_path

def skidpad_path_centre_end(track):
    """Generate skidpad path from centre to end
    Args:
        track (id_track_utils.src.id_track_utils.track.Track): track of the circuit
    Returns:
        generated_path (list(path)): ordered list of midpoints
    """    
    # TODO: finish this function. 
    # Should follow a structure similar to skidpad_path_beg_centre
    pass

track = Track.load_from_file("/home/mario/Driverless/src/path_generators/csv/skidpad.csv")

# Get the coordinates of where the car is located
car_pos = CarPos(track.start_x, track.start_y)
print("track.start_x", track.start_x)
print("track.start_y", track.start_y)

# Generate a path from the beginning to the centre
generated_path = skidpad_path_beg_centre(track)
# plot_track(track, generated_path, car_pos)

# move the car to the centre
centre = generated_path[-2]
car_pos.x = generated_path[-2][0]
car_pos.y = generated_path[-2][1]
print(car_pos)
# plot_track(track, generated_path, car_pos)

# Generate a path from the centre around the left circle
generated_path = skidpad_path_left_circle(track)
# plot_track(track, generated_path, car_pos)

# move the car the the centre
car_pos.x = centre[0]
car_pos.y = centre[1]
plot_track(track, generated_path, car_pos)

# Generate a path from the centre to the end
# TODO: finish this function
# generated_path = skidpad_path_centre_end(track)