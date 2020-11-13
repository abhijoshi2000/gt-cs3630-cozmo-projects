# Main Running Class
import threading
import cozmo
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
    global g_x, g_y, g_a

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

    pf = ParticleFilter(grid)

    has_cube = False
    first_time = True

    ###################

    # ENTER OUR CODE HERE
    # 0:localize
    # 1:pick_up
    # 2:drop_off

    state = 0

    while state != -1:
        await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
        while state == 0: # localize
            has_converged = False
            # Try to converge
            marker_list, annotated_image = await marker_processing(robot, camera_settings)
            x, y, a, has_converged = pf.update(
                compute_odometry(robot.pose), marker_list)
            update_gui(x, y, a, has_converged, annotated_image)
            g_x = x
            g_y = y
            g_a = a
            last_pose = robot.pose
            if not has_converged:
                await robot.drive_wheels(-10.00, 2.00)
            else:
                if not has_cube: # Pick-up
                    state = 1
                else: # Dropoff
                    state = 2
        if state == 1: # Go to pickup
            if cozmo_rrt(g_x, g_y, g_a, 160, 300, 135, robot):
                # Pickup Cube Code
                lookaround = robot.start_behavior(
                cozmo.behavior.BehaviorTypes.LookAroundInPlace)
                cubes = robot.world.wait_until_observe_num_objects(
                    num=1, object_type=cozmo.objects.LightCube, timeout=60)
                lookaround.stop()
                pickup_cube = robot.pickup_object(cubes[0], num_retries=3)
                pickup_cube.wait_for_completed()
                has_cube = True
                state = 2
            else:
                state = 0
        elif state == 2: # Go to drop off
            if cozmo_rrt(g_x, g_y, g_a, 450, 220, 45, robot):
                # Dropoff Cube Code
                place_on_ground = robot.place_object_on_ground_here(
                    cubes[0], num_retries=3)
                place_on_ground.wait_for_completed()
                drive_backward = robot.drive_straight(distance_mm(-325.0),
                                                    speed_mmps(50.0))
                drive_backward.wait_for_completed()
                has_cube = False
                state = 1
            else:
                state = 0
    ###################

async def cozmo_rrt(x_i, y_i, a_i, goal_x, goal_y, goal_a, robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    cozmo_start_x = robot.pose.position.x
    cozmo_start_y = robot.pose.position.y
    cozmo_start_ang = robot.pose.rotation.angle_z.degrees
    x_init = x_i
    y_init = y_i
    ang_init = a_i

    map_width, map_height = cmap.get_size()

    cmap.add_goal(Node((goal_x, goal_y)))

    #reset the current stored paths in cmap
    cmap.reset_paths()

    # Update the current Cozmo position (cozmo_pos and cozmo_angle) to be new node position and angle
    cozmo_pos = Node((x_init + robot.pose.position.x - cozmo_start_x, y_init + robot.pose.position.y - cozmo_start_y))

    # Set new start position for replanning with RRT
    cmap.set_start(cozmo_pos)

    #call the RRT function using your cmap as input, and RRT will update cmap with a new path to the target from the start position
    RRT(cmap, cmap.get_start())

    #get path from the cmap
    path = cmap.get_smooth_path()

    #So initialize "marked" to be an empty dictionary and "update_cmap" = False
    markedCubes = {}
    update_cmap = False

    #while the current cosmo position is not at the goal:
    arrived = False
    i = 0
    while not arrived:
        await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
        #break if path is none or empty, indicating no path was found
        if not path or len(path) == 0 or i >= len(path):
            break

        # Get the next node from the path
        # drive the robot to next node in path. #First turn to the appropriate angle, and then move to it
        next_node = path[i]
        i = i+1

        cozmo_pos = Node((x_init + robot.pose.position.x - cozmo_start_x, y_init + robot.pose.position.y - cozmo_start_y))
        ang_head = robot.pose.rotation.angle_z.degrees - cozmo_start_ang
        await turn_and_move(cozmo_pos, ang_head, next_node, robot)

        marker_list, annotated_image = await marker_processing(robot, camera_settings)
        x, y, a, has_converged = pf.update(
            compute_odometry(robot.pose), marker_list)
        update_gui(x, y, a, has_converged, annotated_image)
        g_x = x
        g_y = y
        g_a = a
        if not has_converged:
            return False
        #await robot.go_to_pose(cozmo.util.Pose(goal_x, goal_y, 0, angle_z=cozmo.util.degrees(goal_a)), relative_to_robot=False).wait_for_completed()
        # Update the current Cozmo position (cozmo_pos and cozmo_angle) to be new node position and angle
        cozmo_pos = Node((x_init + robot.pose.position.x - cozmo_start_x, y_init + robot.pose.position.y - cozmo_start_y))

        # Set new start position for replanning with RRT
        cmap.set_start(cozmo_pos)
        # # detect any visible obstacle cubes and update cmap
        # do_reset, center_goal, markedCubes = await detect_cube_and_update_cmap(robot, markedCubes, cozmo_pos)

        # #if we detected a cube, indicated by update_cmap, reset the cmap path, recalculate RRT, and get new paths
        # if do_reset:
        #     print('RESET')
        #     cmap.reset_paths()
        #     cmap.clear_smooth_path()
        #     cmap.clear_nodes()
        #     cmap.clear_node_paths()
        #     i = 0
        #     RRT(cmap, cozmo_pos)
        #     path = cmap.get_smooth_path()

    return True
    ########################################################################

async def turn_and_move(cozmo_pos, ang_head, goal_pos, robot: cozmo.robot.Robot):
    print('1X: ', cozmo_pos.x, '1Y: ', cozmo_pos.y)
    print('2X: ', goal_pos.x, '2Y: ', goal_pos.y)
    x_coord = goal_pos.x - cozmo_pos.x
    y_coord = goal_pos.y - cozmo_pos.y
    print('dX: ', x_coord, 'dY: ', y_coord)
    print(ang_head)
    ang_head = cozmo.util.degrees(np.arctan2(y_coord, x_coord) / math.pi * 180 - ang_head)
    print(np.arctan2(y_coord, x_coord) / math.pi * 180)
    print(ang_head)
    await robot.turn_in_place(ang_head).wait_for_completed()

    dist = x_coord ** 2 + y_coord ** 2
    dist = dist ** .5
    print('dist',dist)
    await robot.drive_straight(cozmo.util.distance_mm(dist), cozmo.util.Speed(100)).wait_for_completed()


def update_gui(x, y, z, has_converged, annotated_image):
    gui.show_particles(pf.particles)
    gui.show_mean(x, y, z, has_converged)
    gui.show_camera_image(annotated_image)
    gui.updated.set()


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

MAX_NODES = 20000


def step_from_to(node0, node1, limit=75):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    ############################################################################

    #############################################################################
    # Instructors Solution
    if get_dist(node0, node1) < limit:
        return node1
    else:
        theta = np.arctan2(node1.y - node0.y, node1.x - node0.x)
        return Node((node0.x + limit * np.cos(theta), node0.y + limit * np.sin(theta)))


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    ############################################################################

    #############################################################################
    # Instructors Solution
    if np.random.rand() < 0.05:
        # if np.random.rand() < 0.00:
        return Node((cmap.get_goals()[0].x, cmap.get_goals()[0].y))

    else:
        while True:
            rand_node = Node((np.random.uniform(cmap.width),
                              np.random.uniform(cmap.height)))
            if cmap.is_inbound(rand_node) \
                    and (not cmap.is_inside_obstacles(rand_node)):
                break
        return rand_node
    ############################################################################


def RRT(cmap, start):
    # cmap.add_node(start)
    # map_width, map_height = cmap.get_size()
    # while (cmap.get_num_nodes() < MAX_NODES):
    #     ########################################################################
    #     # TODO: please enter your code below.
    #     # 1. Use CozMap.get_random_valid_node() to get a random node. This
    #     #    function will internally call the node_generator above
    #     # 2. Get the nearest node to the random node from RRT
    #     # 3. Limit the distance RRT can move
    #     # 4. Add one path from nearest node to random node
    #     #
    #     rand_node = None
    #     nearest_node = None
    #     pass
    #     ########################################################################
    #     time.sleep(0.01)
    #     cmap.add_path(nearest_node, rand_node)
    #     if cmap.is_solved():
    #         break

    # path = cmap.get_path()
    # smoothed_path = cmap.get_smooth_path()

    # if cmap.is_solution_valid():
    #     print("A valid solution has been found :-) ")
    #     print("Nodes created: ", cmap.get_num_nodes())
    #     print("Path length: ", len(path))
    #     print("Smoothed path length: ", len(smoothed_path))
    # else:
    #     print("Please try again :-(")

    ############################################################################
    # instructors solution
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        rand_node = node_generator(cmap)
        #rand_node = cmap.get_random_valid_node()
        nearest_node_dist = np.sqrt(map_height ** 2 + map_width ** 2)
        nearest_node = None
        for node in cmap.get_nodes():
            if get_dist(node, rand_node) < nearest_node_dist:
                nearest_node_dist = get_dist(node, rand_node)
                nearest_node = node
        rand_node = step_from_to(nearest_node, rand_node)
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")


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
    local_vec = np.array([[node.x], [node.y], [1]])
    global_T_local = np.array([[np.cos(local_angle), -np.sin(local_angle), local_origin.x],
                               [np.sin(local_angle), np.cos(
                                   local_angle), local_origin.y],
                               [0, 0, 1]])
    global_vec = global_T_local.dot(local_vec)
    return Node((int(global_vec[0]), int(global_vec[1])))


async def detect_cube_and_update_cmap(robot, marked, cozmo_pos):
    """Helper function used to detect obstacle cubes and the goal cube.
       1. When a valid goal cube is detected, old goals in cmap will be cleared and a new goal corresponding to the approach position of the cube will be added.
       2. Approach position is used because we don't want the robot to drive to the center position of the goal cube.
       3. The center position of the goal cube will be returned as goal_center.

        Arguments:
        robot -- provides the robot's pose in G_Robot
                 robot.pose is the robot's pose in the global coordinate frame that the robot initialized (G_Robot)
                 also provides light cubes
        cozmo_pose -- provides the robot's pose in G_Arena
                 cozmo_pose is the robot's pose in the global coordinate we created (G_Arena)
        marked -- a dictionary of detected and tracked cubes (goal cube not valid will not be added to this list)

        Outputs:
        update_cmap -- when a new obstacle or a new valid goal is detected, update_cmap will set to True
        goal_center -- when a new valid goal is added, the center of the goal cube will be returned
    """
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 40.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue

        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians

        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(
            object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(
            object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(
            object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(
            object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center, marked


class CozmoThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        # Cozmo can stay on his charger
        cozmo.robot.Robot.drive_off_charger_on_connect = False
        cozmo.run_program(delivery, use_viewer=False)


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(delivery, use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset_paths()
        stopevent.set()


if __name__ == '__main__':\

    global cmap, stopevent
    stopevent = threading.Event()

    # init cozmo thread
    # cozmo_thread = CozmoThread()
    # cozmo_thread.start()
    # gui.show_particles(pf.particles)
    # gui.show_mean(0, 0, 0)
    # gui.start()

    # init robot thread
    robot_thread = RobotThread()
    robot_thread.start()

    # init cmap
    cmap = CozMap("maps/map2.json", node_generator)

    # init gui and visualizer
    print("Rendering GUI and Visualizer")
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
