import rclpy
import pygame

from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from interfaces.msg import CameraView as ICameraView

from commander.camera import Viewer

BASE_CELL_SIZE = 10
VIEW_STEP_SCALE = 10
ZOOM_FACTOR = 1.2

class CameraView:
    def __init__(self, screen_size, cell_size=BASE_CELL_SIZE, zoom=1.0, min_zoom=0.1, max_zoom=10.0):
        self.screen_width, self.screen_height = screen_size
        self.cell_size = cell_size
        self.zoom = zoom
        self.min_zoom = min_zoom
        self.max_zoom = max_zoom

        self.pos = pygame.Vector2(0, 0)

    def world_to_screen(self, world_pos):
        wx, wy = world_pos
        sx = (wx - self.pos.x) * self.zoom + self.screen_width * 0.5
        sy = (wy - self.pos.y) * self.zoom + self.screen_height * 0.5
        return pygame.Vector2(sx, sy)

    def screen_to_world(self, screen_pos):
        sx, sy = screen_pos
        wx = (sx - self.screen_width * 0.5) / self.zoom + self.pos.x
        wy = (sy - self.screen_height * 0.5) / self.zoom + self.pos.y
        return pygame.Vector2(wx, wy)

    def move(self, dx, dy):
        self.pos.x += dx
        self.pos.y += dy

    def set_pos(self, world_pos):
        self.pos = pygame.Vector2(world_pos)

    def zoom_in(self, factor):
        new_zoom = self.zoom * factor
        if new_zoom > self.max_zoom:
            new_zoom = self.max_zoom
        self.zoom = new_zoom

    def zoom_out(self, factor):
        new_zoom = self.zoom / factor
        if new_zoom < self.min_zoom:
            new_zoom = self.min_zoom
        self.zoom = new_zoom

class Direction(IntEnum):
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,

class View:
    def __init__(self):
        self.node = Node("Grid", namespace="Viewer")
        self.node.declare_parameter('width', 1000)
        self.node.declare_parameter('height', 1000)
        self.node.declare_parameter('step', 3)
        
        self.viewer =  Viewer()
        self.viewer.props_update_callback = self.on_props_update
        self.viewer.camera_view_bumps_callback = self.on_view_bumps

        self.update_params()

        self.post_param_callback = self.node.add_post_set_parameters_callback(self.on_post_param_set)

        self.grid_color = (200, 200, 200)
        self.bg_color = (30, 30, 30)

        self.running = True

        self.coords = set()

        self.center_on(self.viewer.props['pos_x'], self.viewer.props['pos_y'])

    def __del__(self):
        self.node.destroy_node()
        self.viewer.destroy_node()

    def get_camera_pov_radius(self):
        return (BASE_CELL_SIZE * VIEW_STEP_SCALE) * self.viewer.props['view_step']
    
    def get_bot_pos(self) -> tuple[float, float]:
        return (self.viewer.props['pos_x'], self.viewer.props['pos_y'])
    
    def on_post_param_set(self, params):
        self.update_params()
        params

    def update_params(self):
        self.width = self.node.get_parameter('width').get_parameter_value().integer_value
        self.height = self.node.get_parameter('height').get_parameter_value().integer_value
        self.step = self.node.get_parameter('step').get_parameter_value().integer_value

        self.camera_view = CameraView((self.width, self.height), cell_size=BASE_CELL_SIZE, zoom=3.0)
        self.screen = pygame.display.set_mode((self.width, self.height))

    def on_view_bumps(self, data: ICameraView):
        for (x, y, bump) in zip(data.x_cords, data.y_cords, data.bumps):
            if (x, y, bump) in self.coords: continue
            self.node.get_logger().info('{}, {}: {}'.format(x, y, bump))
            self.coords.add((x, y, bump))

    def on_props_update(self, props: dict):
        self.center_on(props["pos_x"], props["pos_y"])

    def next_frame(self, dt: float):
        SCROLL_UP_BTN = 4
        SCROLL_DOWN_BTN = 5
        rclpy.spin_once(self.viewer, timeout_sec=0)
        rclpy.spin_once(self.node, timeout_sec=0)

        for ev in pygame.event.get():
            match ev.type:
                case pygame.QUIT:
                    self.running = False
            if ev.type == pygame.QUIT:
                self.running = False

            elif ev.type == pygame.MOUSEBUTTONDOWN:
                if ev.button == SCROLL_UP_BTN:
                    self.set_view_step(self.viewer.props['view_step'] + 0.1)
                elif ev.button == SCROLL_DOWN_BTN:
                    self.set_view_step(self.viewer.props['view_step'] - 0.1)

            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_w:
                    self.move(Direction.UP)
                elif ev.key == pygame.K_s:
                    self.move(Direction.DOWN)
                elif ev.key == pygame.K_a:
                    self.move(Direction.LEFT)
                elif ev.key == pygame.K_d:
                    self.move(Direction.RIGHT)
                elif ev.key == pygame.K_x:
                    self.viewer.cancel_moving()
                elif ev.key == pygame.K_UP:
                    self.camera_view.zoom_in(ZOOM_FACTOR)
                elif ev.key == pygame.K_DOWN:
                    self.camera_view.zoom_out(ZOOM_FACTOR)

        self.screen.fill(self.bg_color)
        self.draw_grid()

        dt

    def draw_grid(self):
        cam = self.camera_view
        cell = cam.cell_size

        top_left = cam.screen_to_world((0, 0))
        bottom_right = cam.screen_to_world((cam.screen_width, cam.screen_height))

        x_start = int((top_left.x // cell)) - 1
        x_end = int((bottom_right.x // cell)) + 1
        y_start = int((top_left.y // cell)) - 1
        y_end = int((bottom_right.y // cell)) + 1

        for x, y, bump in self.coords:
            if x < x_start or y < y_start or x > x_end or y > y_end:
                continue

            if bump > 0.5:
                color = (0, 255, 0, 255)
            elif bump < 0.0:
                color = (139, 69, 19)
            else:
                color = (245, 222, 179)
            self.fill_cell((x, y), color)


        for i in range(x_start, x_end + 1):
            wx = i * cell
            start = cam.world_to_screen((wx, top_left.y))
            end = cam.world_to_screen((wx, bottom_right.y))

            pygame.draw.line(self.screen, (255, 255, 255), (start.x, start.y), (start.x, end.y), 2)

        for j in range(y_start, y_end + 1):
            wy = j * cell
            start = cam.world_to_screen((top_left.x, wy))
            end = cam.world_to_screen((bottom_right.x, wy))

            pygame.draw.line(self.screen, (255, 255, 255), (start.x, start.y), (end.x, start.y), 2)
        
        self.fill_cell(self.get_bot_pos(), (255, 0, 0)) # cam pos
        self.draw_pov() # pov circle

    def fill_cell(self, cell_indices, color):
        cam = self.camera_view
        i, j = cell_indices
        cell_world_size = cam.cell_size

        wx = i * cell_world_size
        wy = j * cell_world_size

        screen_tl = cam.world_to_screen((wx, wy))
        screen_size = cell_world_size * cam.zoom

        rect = pygame.Rect(
            screen_tl.x,
            screen_tl.y,
            screen_size,
            screen_size
        )

        pygame.draw.rect(self.screen, color, rect)
    
    def draw_pov(self):
        i, j = self.get_bot_pos()
        cell_size = self.camera_view.cell_size

        world_center_x = i * cell_size + cell_size / 2
        world_center_y = j * cell_size + cell_size / 2
        world_center = (world_center_x, world_center_y)

        screen_center = self.camera_view.world_to_screen(world_center)

        radius = self.get_camera_pov_radius() * self.camera_view.zoom
        radius = int(radius)
        center_px = (int(screen_center.x), int(screen_center.y))

        # Draw circle outline
        pygame.draw.circle(self.screen, (255, 0, 0), center_px, radius, 2)

    def center_on(self, i, j):
        cell_size = self.camera_view.cell_size
        world_x = i * cell_size + cell_size / 2
        world_y = j * cell_size + cell_size / 2
        self.camera_view.set_pos((world_x, world_y))

    def move(self, dir: Direction):
        self.viewer.move_bot(int(dir), self.step)
    
    def set_view_step(self, value: float):
        req = SetParameters.Request()
        req.parameters = [Parameter('view_step', Parameter.Type.DOUBLE, value).to_parameter_msg()]

        self.viewer.camera_set_param.call_async(req)

def main(args=None):
    pygame.init()
    pygame.display.set_caption('Commander View')

    rclpy.init(args=args)

    grid = View()

    clock = pygame.time.Clock()

    while grid.running:
        dt = clock.tick(60) / 1000.0
        grid.next_frame(dt)

        if not rclpy.ok():
            grid.running = False

        pygame.display.flip()

    del grid
    rclpy.shutdown()

    pygame.quit()

if __name__ == '__main__':
    main()
