import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd

class ReactiveSolver(Node):
    def __init__(self):
        super().__init__('reactive_solver_client')
        self.cli = self.create_client(MoveCmd, 'move_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando o serviço move command...')
        self.req = MoveCmd.Request()
        self.future = None
        self.solved = False
        self.path = []
        self.visited_positions = set()
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        if self.solved:
            return

        if self.future is None or self.future.done():
            if self.future is not None and self.future.result() is not None:
                response = self.future.result()
                self.process_response(response)
            elif self.future is None:
                self.req.direction = 'down'
                self.future = self.cli.call_async(self.req)

    def process_response(self, response):
        left = response.left
        down = response.down
        up = response.up
        right = response.right
        robot_pos = tuple(response.robot_pos)
        target_pos = tuple(response.target_pos)

        self.get_logger().info(f'posicao robo {robot_pos}')
        self.get_logger().info(f'posicao do alvo: {target_pos}')
        self.get_logger().info(f'esquerda: {left}, direita: {right}, cima: {up}, baixo: {down}')

        if robot_pos == target_pos:
            self.solved = True
            self.get_logger().info('Alvo encontradooo')
            rclpy.shutdown()
            return

        self.visited_positions.add(robot_pos)
        self.get_logger().info(f'Posições visitadas: {self.visited_positions}')

        if not self.path or self.path[-1] != robot_pos:
            self.path.append(robot_pos)
            self.get_logger().info(f'Caminho atual: {self.path}')

        if not response.success:
            self.get_logger().info(f'Movimento {self.req.direction} não deu certo')
            blocked_pos = self.get_new_position(robot_pos, self.req.direction)
            self.visited_positions.add(blocked_pos)
            self.get_logger().info(f'Posição bloqueada adicionada: {blocked_pos}')

        possible_moves = self.get_possible_moves(left, right, up, down, robot_pos)
        self.get_logger().info(f'Movimentos possíveis: {possible_moves}')

        if possible_moves:
            move_direction, new_pos = possible_moves[0]
            self.req.direction = move_direction
            self.future = self.cli.call_async(self.req)
            self.get_logger().info(f'Movendo {move_direction}')
        else:
            if self.path:
                self.path.pop()
            if not self.path:
                self.get_logger().info('Nenhum caminho possível f, fechando')
                rclpy.shutdown()
                return
            prev_pos = self.path[-1]
            dx = prev_pos[0] - robot_pos[0]
            dy = prev_pos[1] - robot_pos[1]
            self.get_logger().info(f'Backtracking para: {prev_pos}, dx: {dx}, dy: {dy}')
            move_direction = self.get_direction_from_deltas(dx, dy)
            if move_direction is None:
                self.get_logger().info('Erro ao determinar movimento de backtracking.')
                rclpy.shutdown()
                return
            self.req.direction = move_direction
            self.future = self.cli.call_async(self.req)
            self.get_logger().info(f'Backtracking movendo {move_direction}')

    def get_possible_moves(self, left, right, up, down, robot_pos):
        possible_moves = []
        x, y = robot_pos

        new_pos = (x, y - 1)
        if (left == 'f' or left == 't') and new_pos not in self.visited_positions:
            possible_moves.append(('left', new_pos))
        new_pos = (x, y + 1)
        if (right == 'f' or right == 't') and new_pos not in self.visited_positions:
            possible_moves.append(('right', new_pos))
        new_pos = (x - 1, y)
        if (up == 'f' or up == 't') and new_pos not in self.visited_positions:
            possible_moves.append(('up', new_pos))
        new_pos = (x + 1, y)
        if (down == 'f' or down == 't') and new_pos not in self.visited_positions:
            possible_moves.append(('down', new_pos))

        return possible_moves

    def get_new_position(self, pos, direction):
        x, y = pos
        if direction == 'left':
            return (x, y - 1)
        elif direction == 'right':
            return (x, y + 1)
        elif direction == 'up':
            return (x - 1, y)
        elif direction == 'down':
            return (x + 1, y)
        else:
            return pos


    def get_direction_from_deltas(self, dx, dy):
        if dx == -1 and dy == 0:
            return 'up'
        elif dx == 1 and dy == 0:
            return 'down'
        elif dx == 0 and dy == -1:
            return 'left'
        elif dx == 0 and dy == 1:
            return 'right'
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    reactive_solver = ReactiveSolver()
    rclpy.spin(reactive_solver)
    reactive_solver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
