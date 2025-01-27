# If you run into an "[NSApplication _setup] unrecognized selector" problem on macOS,
# try uncommenting the following snippet

# try:
#     import matplotlib
#     matplotlib.use('TkAgg')
# except ImportError:
#     pass

from skimage import color
import cozmo
import numpy as np
from numpy.linalg import inv
import threading
import time
import sys
import asyncio
from PIL import Image

from markers import detect, annotator

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *

# particle filter functionality


class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom, self.grid)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(
            self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)


# tmp cache
last_pose = cozmo.util.Pose(0, 0, 0, angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False

# goal location for the robot to drive to, (x, y, theta)
goal = (6, 10, 0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid, show_camera=True)
pf = ParticleFilter(grid)


def compute_odometry(curr_pose, cvt_inch=True):
    '''
    Compute the odometry given the current pose of the robot (use robot.pose)

    Input:
        - curr_pose: a cozmo.robot.Pose representing the robot's current location
        - cvt_inch: converts the odometry into grid units
    Returns:
        - 3-tuple (dx, dy, dh) representing the odometry
    '''

    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees

    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / grid.scale, dy / grid.scale

    return (dx, dy, diff_heading_deg(curr_h, last_h))


async def marker_processing(robot, camera_settings, show_diagnostic_image=False):
    '''
    Obtain the visible markers from the current frame from Cozmo's camera.
    Since this is an async function, it must be called using await, for example:

        markers, camera_image = await marker_processing(robot, camera_settings, show_diagnostic_image=False)

    Input:
        - robot: cozmo.robot.Robot object
        - camera_settings: 3x3 matrix representing the camera calibration settings
        - show_diagnostic_image: if True, shows what the marker detector sees after processing
    Returns:
        - a list of detected markers, each being a 3-tuple (rx, ry, rh)
          (as expected by the particle filter's measurement update)
        - a PIL Image of what Cozmo's camera sees with marker annotations
    '''

    global grid

    # Wait for the latest image from Cozmo
    image_event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # Convert the image to grayscale
    image = np.array(image_event.image)
    image = color.rgb2gray(image)

    # Detect the markers
    markers, diag = detect.detect_markers(
        image, camera_settings, include_diagnostics=True)

    # Measured marker list for the particle filter, scaled by the grid scale
    marker_list = [marker['xyh'] for marker in markers]
    marker_list = [(x/grid.scale, y/grid.scale, h) for x, y, h in marker_list]

    # Annotate the camera image with the markers
    if not show_diagnostic_image:
        annotated_image = image_event.image.resize(
            (image.shape[1] * 2, image.shape[0] * 2))
        annotator.annotate_markers(annotated_image, markers, scale=2)
    else:
        diag_image = color.gray2rgb(diag['filtered_image'])
        diag_image = Image.fromarray(
            np.uint8(diag_image * 255)).resize((image.shape[1] * 2, image.shape[0] * 2))
        annotator.annotate_markers(diag_image, markers, scale=2)
        annotated_image = diag_image

    return marker_list, annotated_image


async def run(robot: cozmo.robot.Robot):

    global flag_odom_init, last_pose
    global grid, gui, pf

    # start streaming
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    # Obtain the camera intrinsics matrix
    fx, fy = robot.camera.config.focal_length.x_y
    cx, cy = robot.camera.config.center.x_y
    camera_settings = np.array([
        [fx,  0, cx],
        [0, fy, cy],
        [0,  0,  1]
    ], dtype=np.float)

    ###################

    reached_goal = False
    has_converged = False
    played_happy = False

    while True:
        # Reset robot
        await robot.set_lift_height(0).wait_for_completed()
        await robot.set_head_angle(cozmo.util.degrees(15)).wait_for_completed()

        # Two broad cases - if the robot is not localized, and if it is localized
        if not has_converged:
            # Check if the robot is picked up - if so, relocalize
            if robot.is_picked_up:
                await robot.say_text("I've been kidnapped!").wait_for_completed()
                pf = ParticleFilter(grid)
                continue
            # Else try to converge
            marker_list, annotated_image = await marker_processing(robot, camera_settings)
            x, y, z, has_converged = pf.update(
                compute_odometry(robot.pose), marker_list)
            update_gui(x, y, z, has_converged, annotated_image)
            last_pose = robot.pose
            if not has_converged:
                await robot.drive_wheels(-10.00, 2.00)
        else:
            # If the robot is localized - has_converged = True
            update_gui(x, y, z, has_converged, annotated_image)
            # If the robot is at the goal state and then is picked up
            if reached_goal:
                if robot.is_picked_up: # Robot picked up, relocalize
                    await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabUnhappy).wait_for_completed() # Unhappy animation if the robot is picked up
                    pf = ParticleFilter(grid)
                    has_converged = False
                    reached_goal = False
                    played_happy = False
                    continue

                if played_happy == False:
                    await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabHappy).wait_for_completed()
                    played_happy = True
            # If the robot is not at the goal state and then is picked up
            else:
                if robot.is_picked_up:
                    await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabUnhappy).wait_for_completed() # Happy animation if the robot is picked up
                    pf = ParticleFilter(grid)
                    continue
                x, y, z, _ = compute_mean_pose(pf.particles)

                robot.stop_all_motors()
                time.sleep(1)

                print("FIRST")
                diffAngle = diff_heading_deg(math.degrees(math.atan2(goal[1] - y, goal[0] - x)), z)
                print(diffAngle)
                await robot.turn_in_place(cozmo.util.degrees(diffAngle)).wait_for_completed()

                print("SECOND")
                drive_distance = (math.sqrt((goal[0] - x) ** 2 + (goal[1] - y) ** 2) * grid.scale)
                print(drive_distance)
                await robot.drive_straight(cozmo.util.distance_mm(drive_distance), cozmo.util.speed_mmps(30)).wait_for_completed()

                print("THIRD")
                diffAngle2 = (diff_heading_deg(goal[2], z + (diff_heading_deg(math.degrees(math.atan2(goal[1] - y, goal[0] - x)), z))))
                print(diffAngle2)
                await robot.turn_in_place(cozmo.util.degrees(diffAngle2), speed=cozmo.util.degrees(95)).wait_for_completed()

                reached_goal = True

    ###################


def update_gui(x, y, z, has_converged, annotated_image):
    gui.show_particles(pf.particles)
    gui.show_mean(x, y, z, has_converged)
    gui.show_camera_image(annotated_image)
    gui.updated.set()


class CozmoThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        # Cozmo can stay on his charger
        cozmo.robot.Robot.drive_off_charger_on_connect = False
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    gui.show_particles(pf.particles)
    gui.show_mean(0, 0, 0)
    gui.start()
