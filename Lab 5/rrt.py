import cozmo
import math
import sys
import time
import random

from cmap import *
from gui import *
from utils import *

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

    if get_dist(node0, node1) < limit:
        return node1
    else:
        x_coord = node1.x - node0.x
        y_coord = node1.y - node0.y
        ang_head = np.arctan2(y_coord, x_coord)
        temp = Node((node0.x + (limit * np.cos(ang_head)), node0.y + (limit * np.sin(ang_head))))
        return temp

    ############################################################################






def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    prob = random.random()
    if prob < 0.05:
        goal_coordinates = cmap.get_goals()[0]
        return Node((goal_coordinates.x, goal_coordinates.y))

    while(rand_node is None):
        rand_node = Node((random.random() * cmap.width, random.random() * cmap.height))
        if (not cmap.is_inbound(rand_node)) or cmap.is_inside_obstacles(rand_node):
            rand_node = None
    return rand_node
    ############################################################################



def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #

        #print('--------------------')
        rand_node = cmap.get_random_valid_node()

        nearest_node = None
        nodes = cmap.get_nodes()
        min_dist = math.inf

        #print(rand_node)
        #print(nodes)

        for n in nodes:
            #print(n)
            curr_dist = get_dist(rand_node, n)
            if curr_dist < min_dist:
                nearest_node = n
                min_dist = curr_dist

        rand_node = step_from_to(nearest_node, rand_node)

        ########################################################################


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



async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions. Potential pseudcode is below

    #assume start position is in cmap and was loaded from emptygrid.json as [50, 35] already
    #assume start angle is 0
    #Add final position as goal point to cmap, with final position being defined as a point that is at the center of the arena
    #you can get map width and map weight from cmap.get_size()
    cozmo_start_x = robot.pose.position.x
    cozmo_start_y = robot.pose.position.y
    cozmo_start_ang = robot.pose.rotation.angle_z.degrees
    x_init = 50
    y_init = 35
    ang_init = 0


    map_width, map_height = cmap.get_size()

    cmap.add_goal(Node((map_width/2, map_height/2)))



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


    #marked and update_cmap are both outputted from detect_cube_and_update_cmap(robot, marked, cozmo_pos).
    #and marked is an input to the function, indicating which cubes are already marked
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
        #drive the robot to next node in path. #First turn to the appropriate angle, and then move to it
        #you can calculate the angle to turn through a trigonometric function
        #print(path)
        next_node = path[i]
        #print(next_node)
        i = i+1

        cozmo_pos = Node((x_init + robot.pose.position.x - cozmo_start_x, y_init + robot.pose.position.y - cozmo_start_y))
        ang_head = robot.pose.rotation.angle_z.degrees - cozmo_start_ang
        await turn_and_move(cozmo_pos, ang_head, next_node, robot)
        # Update the current Cozmo position (cozmo_pos and cozmo_angle) to be new node position and angle
        cozmo_pos = Node((x_init + robot.pose.position.x - cozmo_start_x, y_init + robot.pose.position.y - cozmo_start_y))

        # Set new start position for replanning with RRT
        cmap.set_start(cozmo_pos)
        #detect any visible obstacle cubes and update cmap
        do_reset, center_goal, markedCubes = await detect_cube_and_update_cmap(robot, markedCubes, cozmo_pos)

        #if we detected a cube, indicated by update_cmap, reset the cmap path, recalculate RRT, and get new paths
        if do_reset:
            print('RESET')
            cmap.reset_paths()
            cmap.clear_smooth_path()
            cmap.clear_nodes()
            cmap.clear_node_paths()
            i = 0
            RRT(cmap, cozmo_pos)
            path = cmap.get_smooth_path()

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
    await robot.drive_straight(cozmo.util.distance_mm(dist), cozmo.util.Speed(30)).wait_for_completed()



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
    cube_padding = 80.
    cozmo_padding = 200.

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
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center, marked


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
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


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)): #reads input whether we are running the robot version or not
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        #creates cmap based on empty grid json
        #"start": [50, 35],
        #"goals": [] This is empty
        cmap = CozMap("maps/emptygrid.json", node_generator)
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("maps/map2.json", node_generator)
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
