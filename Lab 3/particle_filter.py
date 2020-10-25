from grid import *
from particle import Particle
from utils import *
import setting
from setting import *
import math
import numpy as np


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []

    # Get the requisite amounts to move from odometry estimate received
    dx, dy, dh = odom

    # Case 1: If the odometry estimate is 0, we do not need to move the particles because the robot isn't moving
    if dx == 0 and dy == 0 and dh == 0:
        return particles

    # Case 2: Standard motion update
    for particle in particles:

        # Get the current particle belief
        x = particle.x
        y = particle.y
        h = particle.h

        # Add Gaussian noise to inject randomness
        x_gauss, y_gauss, h_gauss = add_odometry_noise(
            odom, setting.ODOM_HEAD_SIGMA, setting.ODOM_TRANS_SIGMA)

        # Account for rotational movement
        dx, dy = rotate_point(x_gauss, y_gauss, h)

        # Particle after Gaussian Noise and Rotational Movement
        motion_particle = Particle(x + dx, y + dy, h + h_gauss)
        motion_particles.append(motion_particle)

    return motion_particles

# ------------------------------------------------------------------------


def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update
        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
                                * Note that the robot can see mutliple markers at once, and may not see any one
        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles
        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    measured_particles = particles

    '''
    #replace bad particles
    for part in particles:
        if grid.is_free(part.x, part.y):
            measured_particles.append(part)
        else:
            x, y = grid.random_free_place()
            measured_particles.append(Particle(x,y))
            random_parts += 1
    '''
    #don't change if no new info
    if len(measured_marker_list) == 0:
        return particles

    weights = []
    for part in measured_particles:
        #print('p')

        prob = 1.0
        seen_markers = part.read_markers(grid)
        if not grid.is_free(part.x, part.y):
            weights.append(0)
            continue
        if len(seen_markers) <= 0:
            weights.append(0)
            continue

    # Pair observarble markets by distance
        for measured_marker in measured_marker_list:
            #print('a')
            highest_prob = -0.1
            max_prob_pair = None

            # Current Particle observes fewer markers than the robot
            if len(seen_markers) <= 0:
                break

            for observed_marker in seen_markers:

                distance_between_markers = math.sqrt(measured_marker[0]**2 + measured_marker[1]**2) - math.sqrt(observed_marker[0]**2 + observed_marker[1]**2)

                angle_between_markers = diff_heading_deg(measured_marker[2], observed_marker[2])

                current_prob = np.exp(-(distance_between_markers**2)/(2*MARKER_TRANS_SIGMA**2)
                                   - (angle_between_markers**2)/(2*MARKER_ROT_SIGMA**2))

                if current_prob > highest_prob:
                    highest_prob = current_prob
                    max_prob_pair = observed_marker

            if max_prob_pair != None:
                seen_markers.remove(max_prob_pair)

            prob *= highest_prob
            #print(prob)
        weights.append(prob)

    #normalize
    #print(measured_particles, weights)
    weights = np.divide(weights, np.sum(weights))
    #resample
    rand_percent = 0.03
    random_parts = int(PARTICLE_COUNT*rand_percent)
    new_parts = np.random.choice(measured_particles,
        size = len(measured_particles) - random_parts,
        p=weights)
    new_parts = np.ndarray.tolist(new_parts) + Particle.create_random(random_parts, grid)




    return new_parts

