import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd, GetMap
import heapq

class MappingSolver(Node):
    def __init__(self):
        super().__init__('mapping_solver_client')
        
        self.move_cli = self.create_client(MoveCmd, 'move_command')
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço move cmd')

        self.map_cli = self.create_client(GetMap, 'get_map')
        while not self.map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço getmap')

        self.move_req = MoveCmd.Request()
        self.map_req = GetMap.Request()
        self.future = None
        self.path = []
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.map_received = False
        self.maze = None
        self.robot_pos = None
        self.target_pos = None
        self.shutdown_flag = False

    def timer_callback(self):
        if self.shutdown_flag:
            self.destroy_node()
            rclpy.shutdown()
            return

        if not self.map_received:
            self.future = self.map_cli.call_async(self.map_req)
            self.future.add_done_callback(self.map_response_callback)
            self.map_received = True
        elif self.path:
            next_pos = self.path.pop(0)
            direction = self.get_direction(self.robot_pos, next_pos)
            if direction == 'unknown':
                self.shutdown_flag = True
                return
            self.move_req.direction = direction
            self.future = self.move_cli.call_async(self.move_req)
            self.future.add_done_callback(self.move_response_callback)
            self.get_logger().info(f'Movendo {direction}')
            self.robot_pos = next_pos
        else:
            self.shutdown_flag = True

    def map_response_callback(self, future):
        try:
            response = future.result()
            self.process_map_response(response)
        except Exception as e:
            self.shutdown_flag = True

    def process_map_response(self, response):
        occupancy_grid_flattened = response.occupancy_grid_flattened
        occupancy_grid_shape = response.occupancy_grid_shape

        rows = occupancy_grid_shape[0]
        cols = occupancy_grid_shape[1]
        maze = []
        index = 0
        for i in range(rows):
            row = []
            for j in range(cols):
                if index >= len(occupancy_grid_flattened):
                    self.get_logger().error('Index fora do limite')
                    self.shutdown_flag = True
                    return
                cell = occupancy_grid_flattened[index]
                if cell == 'f':
                    row.append(0)
                elif cell == 'b':
                    row.append(1)
                elif cell == 'r':
                    row.append(0)
                    self.robot_pos = (i, j)
                elif cell == 't':
                    row.append(0)
                    self.target_pos = (i, j)
                else:
                    row.append(1)
                index += 1
            maze.append(row)
        self.maze = maze

        if self.robot_pos is None or self.target_pos is None:
            self.get_logger().error('robo e alvo encontrados')
            self.shutdown_flag = True
            return

        self.get_logger().info(f'Posicao do robo: {self.robot_pos}')
        self.get_logger().info(f'Alvo do robo: {self.target_pos}')

        self.path = self.a_star(self.maze, self.robot_pos, self.target_pos)
        if self.path is None:
            self.shutdown_flag = True
            return
        self.get_logger().info(f'Caminho encontrado com {len(self.path)} passos')

    def move_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error('Move cmd falhou')
                self.get_logger().error('Move cmd falhou')
                self.get_logger().error('Move cmd falhou')
                self.get_logger().error('Move cmd falhou')

                self.shutdown_flag = True
        except Exception as e:
            self.shutdown_flag = True

    def get_direction(self, current_pos, next_pos):
        dx = next_pos[0] - current_pos[0]
        dy = next_pos[1] - current_pos[1]
        if dx == -1 and dy == 0:
            return 'up'
        elif dx == 1 and dy == 0:
            return 'down'
        elif dx == 0 and dy == -1:
            return 'left'
        elif dx == 0 and dy == 1:
            return 'right'
        else:
            return 'unknown'

    def a_star(self, maze, start, goal):
        self.get_logger().info('comecando A*')
        rows, cols = len(maze), len(maze[0])
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, [start]))
        closed_set = set()

        while open_set:
            estimated_total_cost, cost_so_far, current, path = heapq.heappop(open_set)

            if current in closed_set:
                continue

            if current == goal:
                self.get_logger().info('MEta encontrada pela A*')
                return path[1:] 

            closed_set.add(current)
            for neighbor in self.get_neighbors(current, maze):
                if neighbor in closed_set:
                    continue
                new_cost = cost_so_far + 1
                estimated_cost = new_cost + self.heuristic(neighbor, goal)
                heapq.heappush(open_set, (estimated_cost, new_cost, neighbor, path + [neighbor]))

        return None

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, position, maze):
        neighbors = []
        x, y = position
        rows, cols = len(maze), len(maze[0])

        if x > 0 and maze[x - 1][y] == 0:
            neighbors.append((x - 1, y))
        if x < rows - 1 and maze[x + 1][y] == 0:
            neighbors.append((x + 1, y))
        if y > 0 and maze[x][y - 1] == 0:
            neighbors.append((x, y - 1))
        if y < cols - 1 and maze[x][y + 1] == 0:
            neighbors.append((x, y + 1))

        return neighbors

def main(args=None):
    rclpy.init(args=args)
    mapping_solver = MappingSolver()
    rclpy.spin(mapping_solver)
    mapping_solver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
