import math
import time
import rclpy
import matplotlib.pyplot as plt
import numpy as np
import cv2
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class Leader(Node):
    def __init__(self):
        super().__init__('leader')
        self.create_subscription(Float32, '/tb3_1_test', self.callback_tb3_1, 10)
        self.create_subscription(Float32, '/tb3_2_test', self.callback_tb3_2, 10)
        self.create_subscription(Odometry, '/tb3_0/odom', self.callback_odom, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.clbk_map, 10)
        self.goal_pub = self.create_publisher(Point, '/tb3_0/goal', 10)
        self.pub_vel = self.create_publisher(Twist, '/tb3_0/cmd_vel', 10)
        self.map = [[]]
        self.map_resolution = None
        self.line_map = np.zeros((2000,2000))
        self.cell_map = np.zeros((2000,2000))

        plt.ion()
        fig, self.ax = plt.subplots()
        fig2, self.ax2 = plt.subplots()
        fig3, self.ax3 = plt.subplots()
        fig4, self.ax4 = plt.subplots()
        plt.show(block=False)
        
        self.draw_map_timer = self.create_timer(10, self.draw_map)
        self.move_bot_timer = self.create_timer(0.1, self.get_goal)
        self.set_vel_timer = self.create_timer(0.1, self.set_vel)
        self.line_cell = -10
        self.cell_wall = 100
        self.cell_circuits = []
        self.done_circuits = []
        self.start_cell = None
        self.current_circuit = None
        self.bot = self.Bot('tb3_0')
        self.goal_node = None
        self.map_loaded = False
        self.last_node = False

        self.wall_distance = 35 # distance the bot's path is from the wall

    class Cell():
        def __init__(self, coords):
            self.coords = coords
            self.next = ()
            self.last = ()
            self.done = False
        def distance(self, other):
            return math.hypot(self.coords[0] - other.coords[0], self.coords[1] - other.coords[1])

    class Bot():
        def __init__(self, name):
            self.name = name
            self.position = Point()
            self.yaw = 0
            self.goal = Point()

    def world_to_map(self, coords):
        x, y = coords
        x = x / self.map_resolution + len(self.map)/2
        y = y / self.map_resolution + len(self.map[0])/2
        return(int(y),int(x))

    def map_to_world(self, coords):
        x, y = coords
        x = (x - len(self.map)/2) * self.map_resolution #+ self.map_resolution/2
        y = (y - len(self.map[0])/2) * self.map_resolution #+ self.map_resolution/2
        return(y,x)

    def get_goal(self):
        if not self.map_loaded:
            return
        x, y = self.bot.position.x, self.bot.position.y
        if x == 0.0 and y == 0.0:
            return
        if self.goal_node == None:
            self.start_node = None
            closest_circuit = None
            for i, each in enumerate(self.cell_circuits):
                if each not in self.done_circuits:
                    closest = self.cell_circuits[i][0]
                    closest_circuit = each
            cx, cy = self.map_to_world(closest.coords)
            closest_d = math.hypot(cx - x, cy - y)
            
            for circuit in self.cell_circuits:
                if circuit in self.done_circuits:
                    continue
                for cell in circuit:
                    cx, cy = self.map_to_world(cell.coords)
                    distance = math.hypot(cx - x, cy - y)
                    if distance < closest_d:
                        closest = cell
                        closest_d = distance
                        closest_circuit = circuit
            self.start_node = closest
            self.goal_node = closest
            self.current_circuit = closest_circuit
            new_goal = Point()
            new_goal.x, new_goal.y  = self.map_to_world(self.goal_node.coords)
            self.bot.goal = new_goal
            self.get_logger().info(f'new_goal: {new_goal} goal_node.coords: {self.goal_node.coords}')
            new_goal = self.goal_node
            i = 0
            while i < 10:
                self.get_logger().info(f'i: {i} self: {new_goal.coords} next: {new_goal.next.coords} last: {new_goal.last.coords}')
                new_goal = new_goal.next
                i+= 1

        x_goal, y_goal = self.bot.goal.x, self.bot.goal.y
        if abs(x_goal-x) < self.map_resolution/2 and abs(y_goal-y) < self.map_resolution/2:
            self.goal_node = self.goal_node.next
            if self.last_node:
                self.last_node = False
                self.done_circuits.append(self.current_circuit)
                self.goal_node = None
                return
            if self.goal_node == self.start_node:
                self.last_node = True
            new_goal = Point()
            new_goal.x, new_goal.y  = self.map_to_world(self.goal_node.coords)
            self.bot.goal = new_goal
            self.get_logger().info(f'new_goal: {new_goal} goal_node.coords: {self.goal_node.coords}')
        
        self.goal_pub.publish(self.bot.goal)

    def set_vel(self):
        #if not self.move:
        #    return
        if not self.map_loaded:
            return
        vel_msg = Twist()
        rads_away = (self.find_angle() - self.bot.yaw + math.pi) % (2*math.pi) - math.pi 
        vel_msg.angular.z = rads_away/2
        distance = math.hypot(self.bot.goal.x - self.bot.position.x, self.bot.goal.y - self.bot.position.y) 
        if distance < self.map_resolution/2:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
        elif abs(rads_away) < 0.1:
            vel_msg.linear.x = math.tanh(distance)/2
        #else:
        #    vel_msg.linear.x = 0.0
        #self.get_logger().info('pos: ' + str(self.bot.position)  + '\ngoal: ' + str(self.bot.goal))
        #+ ' yaw: ' + str(self.yaw) + ' rads: ' + str(rads_away)
        self.pub_vel.publish(vel_msg)

    def find_angle(self):
        x = self.bot.goal.x - self.bot.position.x
        y = self.bot.goal.y - self.bot.position.y
        return math.atan2(y, x)

    def draw_map(self):
        if not self.map_loaded:
            return
        self.ax.clear()
        self.ax.imshow(self.map, cmap='viridis', origin='lower', vmin=self.line_cell, vmax=10)
        self.ax2.clear()
        self.ax2.imshow(self.map, cmap='viridis', origin='lower', vmin=0, vmax=self.cell_wall)
        self.ax3.clear()
        self.ax3.imshow(self.line_map, cmap='viridis', origin='lower', vmin=0, vmax=1)
        self.ax4.clear()
        self.ax4.imshow(self.cell_map, cmap='viridis', origin='lower', vmin=-1, vmax=1)
        x, y = self.world_to_map((self.bot.position.x, self.bot.position.y))
        self.ax4.plot(y, x, 'ro')
        plt.draw()
        plt.pause(1)

    def callback_tb3_1(self, msg):
        self.get_logger().info(f'[TB3_1] Value: {msg.data}')

    def callback_tb3_2(self, msg):
        print(f'[TB3_2] Value: {msg.data}')
    
    def callback_odom(self, msg):
        self.bot.position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.bot.yaw = euler[2]

    def check_map(self, coords):
            return self.map[coords[0]][coords[1]]
    
    def find_inbetween_cell(self, cell1, cell2):
        neighbour_coords = [(self.wall_distance,0),(0,self.wall_distance),(-self.wall_distance,0),(0,-self.wall_distance)]
        x3 = 0
        y3 = 0
        i = 0
        for each in neighbour_coords:
            x, y = each
            for every in [cell1, cell2]:
                x1, y1 = every.coords
                x2 = x+x1
                y2 = y+y1
                if self.check_map((x2, y2)) == 100:
                    x3 += x2-x
                    y3 += y2-y
                    i+=1
        if i!=2:
            self.get_logger().info(f'i not 2, {cell1.coords} {cell2.coords}')
        return (x3,y3)
            

    def clbk_map(self, msg):
        def crawl(coords):
            x, y = coords
            nw, up, ne, left, _, right, sw, down, se = self.neighbours(x, y, True)
            if self.check_map(left) == 0 and self.check_map(up) == 0:
                return right
            elif self.check_map(left) == 0 and self.check_map(down) == 0:
                return up
            elif self.check_map(right) == 0 and self.check_map(up) == 0:
                return down
            elif self.check_map(right) == 0 and self.check_map(down) == 0:
                return left
            elif self.check_map(left) == 0:
                return up
            elif self.check_map(up) == 0:
                return right
            elif self.check_map(right) == 0:
                return down
            elif self.check_map(down) == 0:
                return left
            elif self.check_map(nw) == 0:
                return up
            elif self.check_map(ne) == 0:
                return right
            elif self.check_map(sw) == 0:
                return left
            elif self.check_map(se) == 0:
                return down
            else:
                print(f'coords: {coords} left: {self.check_map(left)} right: {self.check_map(right)} up: {self.check_map(up)} down: {self.check_map(down)}')
                raise
            
        #self.get_logger().info(f'map: {msg}')
        self.get_logger().info(f'map: {msg.info}')
        self.get_logger().info(f'map: {msg.info.resolution}')
        self.map_resolution = msg.info.resolution
        self.map = msg.data
        self.map = np.reshape(self.map, (2000, 2000))
        # gathering all coords that are on walls
        wall_spaces = []
        for x, each in enumerate(self.map):
            for y, every in enumerate(each):
                if every == self.cell_wall:
                    wall_spaces.append((x, y))
        # gathers all coords that are on walls that are on the outside of the map(where bots can not go)
        outer_walls = []
        cur_coords = (0,int(self.map.shape[0]/2))
        while self.map[cur_coords[0]][cur_coords[1]] == 0:
            cur_coords = (cur_coords[0]+1,cur_coords[1])
        start = cur_coords
        outer_walls.append(start)
        cur_coords = crawl(cur_coords)
        while cur_coords != start:
            outer_walls.append(cur_coords)
            cur_coords = crawl(cur_coords)
        
        
        # finds all walls that are not outer walls and that are touching free spaces
        exposed_wall_cells = []
        self.get_logger().info(f'length of outer_walls: {len(outer_walls)}')
        for each in wall_spaces:
            for x, y in self.neighbours(*each):
                if self.map[x][y] == 0 and each not in outer_walls:
                    exposed_wall_cells.append(each)
                    break
        # marks pixels perpendicular to exposed wall pixels based on how far away they are from the wall
        for each in exposed_wall_cells:
            self.start_recursive(*each, self.wall_distance)
        # finds all cells that are self.wall_distance pixels away from walls, marked by the last code segment
        line_cells = []
        for x, each in enumerate(self.map):
            for y, every in enumerate(each):
                if self.map[x][y] == self.line_cell:
                    bad_cell = False
                    up, left, right, down = list(self.neighbours(x,y))
                    if (self.check_map(up) == -10 or self.check_map(down) == -10) and (self.check_map(left) == -10 or self.check_map(right) == -10):
                        bad_cell = True
                    for x1, y1 in self.neighbours(x,y):
                        if self.map[x1][y1] > 1:
                            bad_cell = True
                            break
                    if not bad_cell:
                        line_cells.append((x,y))
        self.get_logger().info(f'line cells: {len(line_cells)}')
        # sorts all line cells into horizontal, vertical, diagonal-sw->ne, diagonal se->nw
        y_aksis = []
        x_aksis = []
        diag_sw_ne_aksis = []
        diag_se_nw_aksis = []
        hor_line_cells = []
        ver_line_cells = []
        diag_sw_ne_line_cells = []
        diag_se_nw_line_cells = []

        def some_function(line_cells, aksis, dir_line_cells, key):
            for coords in line_cells:
                if key(coords) in aksis:
                    dir_line_cells[aksis.index(key(coords))].append(coords)
                else:
                    aksis.append(key(coords))
                    dir_line_cells.append([coords])
        some_function(line_cells, y_aksis, hor_line_cells, lambda coords: coords[1])
        some_function(line_cells, x_aksis, ver_line_cells, lambda coords: coords[0])
        some_function(line_cells, diag_sw_ne_aksis, diag_sw_ne_line_cells, lambda coords: coords[0]-coords[1])
        some_function(line_cells, diag_se_nw_aksis, diag_se_nw_line_cells, lambda coords: coords[0]+coords[1])
        
        # makes the sorted line cells into actual lines
        hor_lines = []
        ver_lines = []
        diag_sw_ne_lines = []
        diag_se_nw_lines = []
        self.get_logger().info(f'se_nw line cells: {len(diag_se_nw_line_cells)}')
        self.get_logger().info(f'sw_ne line cells: {len(diag_sw_ne_line_cells)}')
        start_time = time.time()

        def extract_lines(cell_groups, sort_key, output_list, compare_key):
            for group in cell_groups:
                sorted_cells = sorted(group, key=sort_key)
                line = [sorted_cells[0]]
                for i in range(len(sorted_cells) - 1):
                    if compare_key(sorted_cells[i+1],sorted_cells[i]):
                        line.append(sorted_cells[i+1])
                    else:
                        if len(line) > 1:
                            output_list.append(line)
                        line = [sorted_cells[i+1]]
                if len(line) > 1:
                    output_list.append(line)

        extract_lines(hor_line_cells, lambda coords: coords[0], hor_lines, lambda cell1, cell2: cell1[0] == cell2[0]+1)
        extract_lines(ver_line_cells, lambda coords: coords[1], ver_lines, lambda cell1, cell2: cell1[1] == cell2[1]+1)
        extract_lines(diag_sw_ne_line_cells, lambda coords: coords[0] - coords[1], diag_sw_ne_lines, 
                      lambda cell1, cell2: cell1[0] == cell2[0]+1 and cell1[1] == cell2[1]+1)
        extract_lines(diag_se_nw_line_cells, lambda coords: coords[0], diag_se_nw_lines, 
                      lambda cell1, cell2: cell1[0] == cell2[0]+1 and cell1[1] == cell2[1]-1)
        self.get_logger().info(f'Finished loop')
        
        all_lines = hor_lines + ver_lines + diag_sw_ne_lines + diag_se_nw_lines
        # shorten lines that are overlapping
        seen = []
        overlapping_lines = []
        dupe_lines = []
        for each in all_lines:
            x = each[0]
            y = each[-1]
            if each in overlapping_lines:
                dupe_lines.append(each)
            elif x in seen or y in seen:
                while x in seen:
                    each.pop(0)
                    if len(each) <= 1:
                        break
                    x = each[0]
                while y in seen:
                    each.pop(-1)
                    if len(each) <= 1:
                        break
                    y = each[-1]
                overlapping_lines.append(each)
            else:
                seen.append(x)
                seen.append(y)
        # filter out short lines
        all_lines = [line for line in all_lines if len(line) > 1]
        # map for visualisation
        for x_lines in [hor_lines, ver_lines, diag_sw_ne_lines, diag_se_nw_lines]:
            for line in x_lines:
                for x, y in line:
                    for x1, y1 in self.neighbours(x,y, True):
                        for x2, y2 in self.neighbours(x1,y1, True):
                            self.line_map[x2][y2] = 1
        self.get_logger().info(f'Finished loop over here also')
        #self.draw_map()
        #debugging info
        unique_overlapping_lines = set(tuple(line) for line in overlapping_lines)
        unique_dupe_lines = set(tuple(line) for line in dupe_lines)
        # makes cells from the line endpoints
        cells = []
        for line in all_lines:
            cell1 = self.Cell(line[0])
            cell2 = self.Cell(line[-1])
            cell1.next = cell2
            cell2.next = cell1
            cells.append(cell1)
            cells.append(cell2)
        # populates map for visualisation
        for each in cells:
            self.cell_map[each.coords[1]][each.coords[0]] = 1

        #debugging    
        dupes = []
        seen = []
        for c in cells:
            if c.coords in seen:
                dupes.append(c)
            else:
                seen.append(c.coords)

        # connects each cell.last to the closest cell
        unfinished_cells = cells
        self.get_logger().info(f'cells: {len(cells)}')
        while len(unfinished_cells) > 0:
            self.get_logger().info(f'unfinished_cells: {len(unfinished_cells)}')
            for i, c in enumerate(unfinished_cells):
                best = None
                best_d = float('inf')
                for j, d in enumerate(unfinished_cells):
                    if c.next.coords == d.coords or c.coords == d.coords or i == j:
                        continue
                    dist = c.distance(d)
                    if dist < best_d:
                        best = d
                        best_d = dist
                consent = True
                if c.last != ():
                    if not c.distance(c.last) < best_d:
                        consent = False
                if best.last != ():
                    if not best.distance(best.last) < best_d:
                        consent = False
                if consent:
                    if c.last != ():
                        c.last.last = ()
                    c.last = best
                    if best.last != ():
                        best.last.last = ()
                    best.last = c
            unfinished_cells = [unfinished_cell for unfinished_cell in cells if unfinished_cell.last == ()]
            #best.last = c


        # makes incomplete circuits based on if cells are connected to eachother
        circuits = []
        for cell in cells:
            if cell.done:
                continue
            circuit = []
            cur = cell
            prev = None
            while cur is not None and not cur.done:
                circuit.append(cur)
                cur.done = True
                if prev is cur.next:
                    next = cur.last
                else:
                    next = cur.next
                prev, cur = cur, next
                if cur is cell:
                    break
            circuits.append(circuit)
        
        # Merge circuits together that have cells connected
        for i, each in enumerate(circuits):
            for j, every in enumerate(circuits):
                if j == i or len(each) == 0 or len(every) == 0:
                    continue
                #self.get_logger().info(f'{[circuits[i][0].coords, circuits[j][-1].next.coords, circuits[j][-1].last.coords]}')
                if circuits[i][0].coords == circuits[j][-1].next.coords or circuits[i][0].coords == circuits[j][-1].last.coords:
                    circuits[j] += circuits[i]
                    circuits[i] = []
                    break
                elif circuits[i][0].coords == circuits[j][0].next.coords or circuits[i][0].coords == circuits[j][0].last.coords:
                    circuits[j].reverse()
                    circuits[j] += circuits[i]
                    circuits[i] = []
                    break
                elif circuits[i][-1].coords == circuits[j][-1].next.coords or circuits[i][-1].coords == circuits[j][-1].last.coords:
                    circuits[i].reverse()
                    circuits[j] += circuits[i]
                    circuits[i] = []
                    break

        # remove leftover circuits that get merged and emptied               
        while circuits.count([]): circuits.remove([]) 
        # make each cell.next point to the next cell in the circuit
        for circuit in circuits:
            for i, cell in enumerate(circuit):
                # lets last cell check first cell
                if i == len(circuit)-1:
                    i = -1 
                if cell.next != circuit[i+1]:
                    last = cell.last
                    cell.last = cell.next
                    cell.next = last

        # make diagonals that are not 45 degrees, into a line
        for coord in [0, 1]:
            for circuit in circuits:
                chains = []
                chain = []
                cell = circuit[2]
                
                while True:
                    if cell.coords[coord] == cell.next.coords[coord]:
                        chain.append(cell)
                        chain.append(cell.next)
                        cell = cell.next.next
                    else:
                        cell = cell.next.next
                        if len(chain) > 2:
                            chains.append(chain)
                        chain = []
                    if cell == circuit[0]:
                        break
            for chain in chains:
                chain = list(chain)
                chain[0].next = chain[-1]
                chain[-1].last = chain[0]
                for cell in chain:
                    if cell == chain[0] or cell == chain[-1]:
                        continue 
                    circuit.remove(cell)
        self.cell_circuits = circuits

        #self.get_logger().info(f'{[[cell.coords, cell.next.coords, cell.last.coords]for cell in cells]}')
        self.get_logger().info(f'circuits: {len(self.cell_circuits)} cells: {len(cells)}')
        #for i, each in enumerate(self.cell_circuits):
            #self.get_logger().info(f'circuit {i}: {[[every.coords, every.next.coords, every.last.coords] for every in each]}')
        self.get_logger().info(f'dupes: {len(dupes)} unique dupes: {len(set(dupes))}')
        self.get_logger().info(f'overlapping_lines: {len(overlapping_lines)} unique overlapping_lines: {len(set(unique_overlapping_lines))}')
        self.get_logger().info(f'dupe_lines: {len(dupe_lines)} unique dupe_lines: {len(set(unique_dupe_lines))}')
        self.cell_map = draw_circuits_on_map(circuits)

        self.map_loaded = True

    def neighbours(self, x, y, corners=False):
        """
        returns (up, left, right, down) if not corners
        returns (nw, n, ne, w, middle, e, sw, s, se) if corners
        """
        additives = ((0,1),(1,0),(-1,0),(0,-1)) # up right left down
        if corners:
            additives = ((-1,1),(0,1),(1,1),(-1,0),(0,0),(1,0),(-1,-1),(0,-1),(1,-1))
        for x1, y1 in additives:
            yield((x+x1, y+y1))
    
    def start_recursive(self, x, y, i):
        neighbour_cells = list(self.neighbours(x,y, True))
        for wall1, wall2, free1, free2 in ((3,5,1,7),(1,7,3,5)):
                if self.map[neighbour_cells[wall1]] == self.cell_wall or self.map[neighbour_cells[wall2]] == self.cell_wall:
                    for free_cell in (free1, free2):
                        if self.map[neighbour_cells[free_cell]] != self.cell_wall:
                            x1, y1 = neighbour_cells[free_cell]
                            self.recursive(*neighbour_cells[free_cell], i-1, x1-x, y1-y)
    def recursive(self, x, y, i, x1, y1):
        if i == 0:
            self.map[x][y] = self.line_cell
            return
        elif i < self.map[x][y]:
            return
        self.map[x][y] = i
        self.recursive(x+x1, y+y1, i-1, x1, y1)

def draw_point_thick(map, x, y, value, thickness=5):
    r = thickness // 2
    for i in range(x - r, x + r + 1):
        for j in range(y - r, y + r + 1):
            if 0 <= i < map.shape[0] and 0 <= j < map.shape[1]:
                map[i, j] = value


def draw_line(mat, x, y, x1, y1, value, thickness=5):
    x_diff = abs(x1 - x)
    y_diff = abs(y1 - y)
    sx = 1 if x < x1 else -1
    sy = 1 if y < y1 else -1
    err = x_diff - y_diff
    while True:
        draw_point_thick(mat, x, y, value, thickness)
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -y_diff:
            err -= y_diff
            x += sx
        if e2 < x_diff:
            err += x_diff
            y += sy

def draw_circuits_on_map(circuits, size=2000, thickness=5):
    map = np.full((size, size), -1)
    for idx, circuit in enumerate(circuits):
        for i in range(len(circuit) - 1):
            x, y = circuit[i].coords
            x1, y1 = circuit[i + 1].coords
            draw_line(map, x, y, x1, y1, idx, thickness)
    return map

def main(args=None):
    rclpy.init(args=args)
    node = Leader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
