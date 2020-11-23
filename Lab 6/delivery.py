# Main Running Class
import threading
import cozmo
from math import sin, cos
from cmap import *
from particle import *
from gui import *
from visualizer import *
from markers import detect, annotator
from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *
from skimage import color
import cozmo
import numpy as np
from numpy.linalg import inv
import threading
import time
import sys
import asyncio
from PIL import Image


async def delivery(robot: cozmo.robot.Robot):

    global flag_odom_init, last_pose
    global grid, gui, pf
    global x, y, a
    global lox, loy, loa

    # start streaming
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()
    await robot.set_lift_height(0).wait_for_completed()

    # Obtain the camera intrinsics matrix
    fx, fy = robot.camera.config.focal_length.x_y
    cx, cy = robot.camera.config.center.x_y
    camera_settings = np.array([
        [fx,  0, cx],
        [0, fy, cy],
        [0,  0,  1]
    ], dtype=np.float)

    pf = ParticleFilter(grid, guess_x=13, guess_y=9)

    has_cube = False
    first_time = True

    ###################

    # ENTER OUR CODE HERE
    # 0:localize
    # 1:pick_up
    # 2:drop_off

    state = 0

    while state != -1:
        await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()
        if state == 0 and not first_time:
                pf = ParticleFilter(grid, guess_x=20, guess_y=9)
        while state == 0: # localize
            has_converged = False
            marker_list, annotated_image = await marker_processing(robot, camera_settings)
            x, y, a, has_converged = pf.update(
                compute_odometry(robot.pose), marker_list)
            update_gui(x, y, a, has_converged, annotated_image)
            x, y, a = x * 25.4, y * 25.4, a
            last_pose = robot.pose
            if not has_converged:
                await robot.drive_wheels(-6.00, 6.00)
            else:
                await robot.drive_wheels(0.00, 0.00)
                print("Localized to:", x, y, a)
                state = 1
        if state == 1: # Go to pickup
            if first_time:
                path =[Node((250, 250)), Node((140, 330))]
            else:
                path = [Node((500, 150)), Node((435, 130)),
                    Node((370, 110)), Node((285, 130)),
                    Node((200, 150)), Node((170, 240)),
                    Node((140, 330))]
            for i in range(len(path)):
                await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()
                # Localize for more certainty
                has_converged = False
                while not first_time and i == len(path) - 1 and not has_converged:
                    has_converged = False
                    marker_list, annotated_image = await marker_processing(robot, camera_settings)
                    x, y, a, has_converged = pf.update(
                        compute_odometry(robot.pose), marker_list)
                    update_gui(x, y, a, has_converged, annotated_image)
                    x, y, a = x * 25.4, y * 25.4, a
                    last_pose = robot.pose
                    if not has_converged:
                        await robot.drive_wheels(-6.00, 6.00)
                    else:
                        await robot.drive_wheels(0.00, 0.00)
                        print("Localized to:", x, y, a)

                marker_list, annotated_image = await marker_processing(robot, camera_settings)
                x, y, a, has_converged = pf.update(
                    compute_odometry(robot.pose), marker_list)
                update_gui(x, y, a, has_converged, annotated_image)
                x, y, a = x * 25.4, y * 25.4, a
                last_pose = robot.pose
                cozmo_pos = Node((x , y))
                ang_head = a
                next_node = path[i]
                print(cozmo_pos.x, cozmo_pos.y, ang_head, next_node.x, next_node.y)
                await turn_and_move(cozmo_pos, ang_head, next_node, robot)
                if first_time and i == (len(path) - 1):
                    await (robot.say_text("Ready for cube")).wait_for_completed()
                    first_time = False

            marker_list, annotated_image = await marker_processing(robot, camera_settings)
            x, y, a, has_converged = pf.update(
                compute_odometry(robot.pose), marker_list)
            update_gui(x, y, a, has_converged, annotated_image)
            x, y, a = x * 25.4, y * 25.4, a
            last_pose = robot.pose

            await (robot.drive_straight(cozmo.util.distance_mm(-80.0),
                                            cozmo.util.speed_mmps(50.0)).wait_for_completed())

            # Pickup Cube Code
            cubes = await (robot.world.wait_until_observe_num_objects(
                num=1, object_type=cozmo.objects.LightCube, timeout=60))
            pickup_cube = await(robot.pickup_object(cubes[0], num_retries=3).wait_for_completed())
            olx, oly, ola = last_pose.position.x, last_pose.position.y, last_pose.rotation.angle_z.degrees
            ogx, ogy, oga = x, y, a
            loa = oga - ola
            lha = np.arctan2(oly, olx)
            lh = olx / cos(lha)
            gha = lha + loa
            lox = ogx - lh * cos(gha)
            loy = ogy - lh * sin(gha)
            has_cube = True
            state = 2
            print("Cube Pickup Angle: ", a)

        elif state == 2: # Go to drop off
            path = [Node((150, 160)), Node((340, 120)), Node((500, 120)), Node((500, 330))]

            for i in range(len(path)):
                if i == 0:
                    nlx, nly, nla = robot.pose.position.x, robot.pose.position.y, robot.pose.rotation.angle_z.degrees
                    ngpos = get_global_node(loa, Node((lox,loy)), Node((nlx, nly)))
                    ngx, ngy = ngpos.x, ngpos.y
                    nga = nla + loa
                    x, y, a = ngx, ngy, nga
                    first_x, first_y = x, y
                else:
                    x, y = path[i-1].x, path[i-1].y
                    if i == 1:
                        print('Pos: ', x, y)
                        print('Was: ', first_x, first_y)
                        a = np.arctan2((y - first_y), (x - first_x)) * 180 / math.pi
                    else:
                        a = np.arctan2((y - path[i-2].y), (x - path[i-2].x))

                cozmo_pos = Node((x, y))
                ang_head = a
                next_node = path[i]

                last_pose = robot.pose
                print("Dropping off ", i, ": ", cozmo_pos.x, cozmo_pos.y, ang_head, next_node.x, next_node.y, robot)
                # await robot.say_text(str(int(i))).wait_for_completed()
                # await robot.say_text(str(int(x))).wait_for_completed()
                # await robot.say_text(str(int(y))).wait_for_completed()
                # await robot.say_text(str(int(next_node.x))).wait_for_completed()
                # await robot.say_text(str(int(next_node.y))).wait_for_completed()
                await turn_and_move(cozmo_pos, ang_head, next_node, robot)

            # Dropoff Cube Code
            last_pose = robot.pose
            olx, oly, ola = last_pose.position.x, last_pose.position.y, last_pose.rotation.angle_z.degrees
            ogx, ogy, oga = x, y, a

            loa = oga - ola
            lha = np.arctan2(oly, olx)
            lh = olx / cos(lha)
            gha = lha + loa
            lox = ogx - lh * cos(gha)
            loy = ogy - lh * sin(gha)

            place_on_ground = await (robot.place_object_on_ground_here(
                cubes[0], num_retries=3).wait_for_completed())
            drive_backward = await (robot.drive_straight(cozmo.util.distance_mm(-40.0),
                                                cozmo.util.speed_mmps(50.0)).wait_for_completed())

            nlx, nly, nla = robot.pose.position.x, robot.pose.position.y, robot.pose.rotation.angle_z.degrees

            ngpos = get_global_node(loa, Node((lox,loy)), Node((nlx, nly)))
            ngx, ngy = ngpos.x, ngpos.y
            nga = nla + loa
            x, y, a = ngx / 25.4, ngy /25.4, nga

            has_cube = False
            state = 0

def get_global_node(local_angle, local_origin, node):
    """Helper function: Transform the node's position (x,y) from local coordinate frame specified by local_origin and local_angle to global coordinate frame.
                        This function is used in detect_cube_and_update_cmap()
        Arguments:
        local_angle, local_origin -- specify local coordinate frame's origin in global coordinate frame
        local_angle -- a single angle value
        local_origin -- a Node object
        Outputs:
        new_node -- a Node object that decribes the node's position in global coordinate frame
    """
    ########################################################################
    # TODO: please enter your code below.

    new_node = None
    t_mat = [
      [np.cos(local_angle), -(np.sin(local_angle)), local_origin.x],
      [np.sin(local_angle), np.cos(local_angle),    local_origin.y],
      [0,                   0,                      1]
    ]
    node_mat = [
      [node.x],
      [node.y],
      [1]
    ]
    new_mat = np.dot(t_mat,node_mat)
    new_node = Node((new_mat[0, 0], new_mat[1, 0]))
    return new_node

    ########################################################################

async def turn_and_move(cozmo_pos, ang_head, goal_pos, robot: cozmo.robot.Robot):
    #await robot.say_text("Ready for movement").wait_for_completed()
    #print('1X: ', cozmo_pos.x, '1Y: ', cozmo_pos.y)
    #print('2X: ', goal_pos.x, '2Y: ', goal_pos.y)
    x_coord = goal_pos.x - cozmo_pos.x
    y_coord = goal_pos.y - cozmo_pos.y
    #print('dX: ', x_coord, 'dY: ', y_coord)
    #print("old facing", ang_head)
    ang_head = np.arctan2(y_coord, x_coord) / math.pi * 180 - ang_head
    #print("da", ang_head)
    ang_head += 180
    ang_head = ang_head % 360
    ang_head -= 180

    ang_head = cozmo.util.degrees(ang_head)
    #print(np.arctan2(y_coord, x_coord) / math.pi * 180)
    #print(ang_head)
    await robot.turn_in_place(ang_head).wait_for_completed()

    dist = x_coord ** 2 + y_coord ** 2
    dist = dist ** .5
    # print('dist',dist)
    await robot.drive_straight(cozmo.util.distance_mm(dist), cozmo.util.speed_mmps(50.0)).wait_for_completed()


def update_gui(x, y, z, has_converged, annotated_image):
    gui.show_particles(pf.particles)
    gui.show_mean(x, y, z, has_converged)
    gui.show_camera_image(annotated_image)
    gui.updated.set()


class ParticleFilter:

    def __init__(self, grid, guess_x=-1, guess_y=-1):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid, guess_x, guess_y)
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

MAX_NODES = 20000

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(delivery, use_3d_viewer=False, use_viewer=False)
        stopevent.set()


if __name__ == '__main__':\

    global cmap, stopevent
    stopevent = threading.Event()

    # init cozmo thread
    robot_thread = RobotThread()
    robot_thread.start()
    gui.show_particles(pf.particles)
    gui.show_mean(0, 0, 0)
    gui.start()

    # init robot thread
    stopevent.set()
