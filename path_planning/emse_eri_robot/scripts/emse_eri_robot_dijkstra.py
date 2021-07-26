#! /usr/bin/env python
import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from gridviz import GridViz
import random
import secrets
import csv

def detect_obstacle_space(index, width, height, costmap):
  """[This function aims at printing the 8 adjacent neighbors to the robot at every step the robot takes in the costmap.]

  Args:
      index ([type]): [description]
      width ([type]): [description]
      height ([type]): [description]
      costmap ([type]): [description]
      orthagonal_step_cost ([type]): [description]
  """

  obstacle_cost = 1 
  """A donating value used to distinct between free space [0] and between obstacle space [1-254]
  """
  space = [] 
  """A list containing the values denoting if obstacle is present there or not.
  """
  
  upper = index - width
  if upper > 0:
    if costmap[upper] < obstacle_cost:
      upper_space = "Free"
      upper_space_coordinate = "(1,2)"
      space.append([upper_space_coordinate, upper_space])
    else:
      upper_space = "Obstacle"
      upper_space_coordinate = "(1,2)"
      space.append([upper_space_coordinate, upper_space])
  
  left = index - 1
  if left % width > 0:
    if costmap[left] < obstacle_cost:
      left_space = "Free"
      left_space_coordinate = "(2,1)"
      space.append([left_space_coordinate, left_space])
    else:
      left_space_coordinate = "(2,1)"
      left_space = "Obstacle"
      space.append([left_space_coordinate, left_space])

  right = index + 1
  if right % width != (width + 1):
    if costmap[right] < obstacle_cost:
      right_space = "Free"
      right_space_coordinate = "(2,3)"
      space.append([right_space_coordinate, right_space])
    else:
      right_space = "Obstacle"
      right_space_coordinate = "(2,3)"
      space.append([right_space_coordinate, right_space])

  lower = index + width
  if lower <= height * width:
    if costmap[lower] < obstacle_cost:
      lower_space = "Free"
      lower_space_coordinate = "(3,2)"
      space.append([lower_space_coordinate, lower_space])
    else:
      lower_space = "Obstacle"
      lower_space_coordinate = "(3,2)"
      space.append([lower_space_coordinate, lower_space])

  upper_left = index - width - 1
  if upper_left > 0 and upper_left % width > 0:
    if costmap[upper_left] < obstacle_cost:
      upper_left_space = "Free"
      upper_left_coordinate = "(1,1)"
      space.append([upper_left_coordinate, upper_left_space])
    else: 
      upper_left_space = "Obstacle"
      upper_left_coordinate = "(1,1)"
      space.append([upper_left_coordinate, upper_left_space])

  upper_right = index - width + 1
  if upper_right > 0 and (upper_right) % width != (width-1):
    if costmap[upper_right] < obstacle_cost:
      upper_right_space = "Free"
      upper_right_coordinate = "(1,3)"
      space.append([upper_right_coordinate, upper_right_space])
    else: 
      upper_right_coordinate = "(1,3)"
      upper_right_space = "Obstacle"
      space.append([upper_right_coordinate, upper_right_space])
  
  lower_left = index + width - 1
  if lower_left < height * width and lower_left & width != 0:
    if costmap[lower_left] < obstacle_cost:
      lower_left_space = "Free"
      lower_left_coordinate = "(3,1)"
      space.append([lower_left_coordinate, lower_left_space])
    else:
      lower_left_coordinate = "(3,1)"
      lower_left_space = "Obstacle"
      space.append([lower_left_coordinate, lower_left_space])

  lower_right = index + width + 1
  if (lower_right) <= height * width and lower_right % width != (width-1):
    if costmap[lower_right] < obstacle_cost:
      lower_right_space = "Free"
      lower_right_coordinate = "(3,3)"
      space.append([lower_right_coordinate, lower_right_space])
    else:
      lower_right_coordinate = "(3,3)"
      lower_right_space = "Obstacle"
      space.append([lower_right_coordinate, lower_right_space])

  return space

def find_neighbors(index, width, height, costmap, orthogonal_step_cost):
  """
  Identifies neighbor nodes inspecting the 4 adjacent neighbors
  Checks if neighbor is inside the map boundaries and if is not an obstacle according to a threshold
  Returns a list with valid neighbour nodes as [index, step_cost] pairs
  """
  neighbors = []
  # length of diagonal = length of one side by the square root of 2 (1.41421)
  diagonal_step_cost = orthogonal_step_cost * 1.41421
  # threshold value used to reject neighbor nodes as they are considered as obstacles [1-254]
  lethal_cost = 1

  secretsGenerator = secrets.SystemRandom()

  upper = (index - width)
  if upper > 0:
    if costmap[upper] < lethal_cost:
      step_cost =upper* orthogonal_step_cost + costmap[upper]/255
      neighbors.append([upper, step_cost])

  left = (index - 1)
  if left % width > 0:
    if costmap[left] < lethal_cost:
      step_cost =  left * orthogonal_step_cost + costmap[left]/255
      neighbors.append([left, step_cost])

  right = (index + 1)
  if right % width != (width + 1):
    if costmap[right] < lethal_cost:
      step_cost =  right * orthogonal_step_cost + costmap[right]/255
      neighbors.append([right, step_cost])

  lower = (index + width)
  if lower <= height * width:
    if costmap[lower] < lethal_cost:
      step_cost = lower * orthogonal_step_cost + costmap[lower]/255
      neighbors.append([lower, step_cost])

  return neighbors

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  ''' 
  # costmap as 1-D array representation
  costmap = req.costmap_ros
  
  # rospy.loginfo(costmap)

  # number of columns in the occupancy grid
  width = req.width
  # number of rows in the occupancy grid
  height = req.height
  start_index = req.start
  goal_index = req.goal
  # side of each grid map square in meters
  resolution = 0.2
  # origin of grid map (bottom left pixel) w.r.t. world coordinates (Rviz's origin)
  origin = [-7.4, -7.4, 0]

  # i = start_index

  # while True:
  #   obstacle_space = detect_obstacle_space(i, width, height, costmap)
  #   with open('/home/whiskygrandee/catkin_ws/src/emse_bot/log/demo-log.csv', 'w', encoding = 'UTF-8') as f:
  #     writer = csv.writer(f)
  #     writer.writerow(obstacle_space)
  #   rospy.loginfo(obstacle_space)
  #   # rospy.loginfo(goal_index - start_index)
  #   i = i + 1

  #   if (i != goal_index):
  #     break

  viz = GridViz(costmap, resolution, origin, start_index, goal_index, width)

  # time statistics
  start_time = rospy.Time.now()

  # calculate the shortes path using Dijkstra
  path = dijkstra(start_index, goal_index, width, height, costmap, resolution, origin, viz)
  rospy.loginfo(path)
  if not path:
    rospy.logwarn("No path returned by Dijkstra's shortes path algorithm")
    path = []
  else:
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('++++++++ Dijkstra execution metrics ++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")

  resp = PathPlanningPluginResponse()
  resp.plan = path
  return resp




def dijkstra(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz):
  ''' 
  Performs Dijkstra's shortes path algorithm search on a costmap with a given start and goal node
  '''

  # create an open_list
  open_list = []

  # set to hold already processed nodes
  closed_list = set()

  # dict for mapping children to parent
  parents = dict()

  # dict for mapping g costs (travel costs) to nodes
  g_costs = dict()

  # set the start's node g_cost
  g_costs[start_index] = 0

  # add start node to open list
  open_list.append([start_index, 0])

  shortest_path = []  

  path_found = False
  rospy.loginfo('Dijkstra: Done with initialization')

  # Main loop, executes as long as there are still nodes inside open_list
  while open_list:

    # sort open_list according to the lowest 'g_cost' value (second element of each sublist)
    open_list.sort(key = lambda x: x[1]) 
    # extract the first element (the one with the lowest 'g_cost' value)
    current_node = open_list.pop(0)[0]

    # Close current_node to prevent from visting it again
    closed_list.add(current_node)

    # Optional: visualize closed nodes
    grid_viz.set_color(current_node,"pale yellow")

    # If current_node is the goal, exit the main loop
    if current_node == goal_index:
      path_found = True
      break

    # Get neighbors of current_node
    neighbors = find_neighbors(current_node, width, height, costmap, resolution)

    # Loop neighbors
    for neighbor_index, step_cost in neighbors:

      # Check if the neighbor has already been visited
      if neighbor_index in closed_list:
        continue

      # calculate g_cost of neighbour considering it is reached through current_node
      g_cost = g_costs[current_node] + step_cost

      # Check if the neighbor is in open_list
      in_open_list = False
      for idx, element in enumerate(open_list):
        if element[0] == neighbor_index:
          in_open_list = True
          break

      # CASE 1: neighbor already in open_list
      if in_open_list:
        if g_cost < g_costs[neighbor_index]:
          # Update the node's g_cost inside g_costs
          g_costs[neighbor_index] = g_cost
          parents[neighbor_index] = current_node
          # Update the node's g_cost inside open_list
          open_list[idx] = [neighbor_index, g_cost]

      # CASE 2: neighbor not in open_list
      else:
        # Set the node's g_cost inside g_costs
        g_costs[neighbor_index] = g_cost
        parents[neighbor_index] = current_node
        # Add neighbor to open_list
        open_list.append([neighbor_index, g_cost])

        # Optional: visualize frontier
        grid_viz.set_color(neighbor_index,'orange')

  rospy.loginfo('Dijkstra: Done traversing nodes in open_list')

  if not path_found:
    rospy.logwarn('Dijkstra: No path found!')
    return shortest_path

  # Reconstruct path by working backwards from target
  if path_found:
      node = goal_index
      shortest_path.append(goal_index)
      while node != start_index:
          shortest_path.append(node)
          # get next node
          node = parents[node]
  # reverse list
  shortest_path = shortest_path[::-1]
  rospy.loginfo(shortest_path)
  rospy.loginfo('Dijkstra: Done reconstructing path')
  return shortest_path




def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

if __name__ == '__main__':
  rospy.init_node('dijkstra_path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  cmd_vel_value = Twist()
  
  rospy.on_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
