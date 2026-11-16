import argparse
from collections import deque
from dataclasses import dataclass, field
import heapq
import json
import math
import random
import sys
from typing import Callable, Optional

import pygame


WIDTH, HEIGHT = 1050, 680
MAP_WIDTH = WIDTH - 260
FPS = 60
CELL = 10
ROAD_TOP, ROAD_HEIGHT = 250, 170
ROAD_BOTTOM = ROAD_TOP + ROAD_HEIGHT
CROSSWALK_LEFT, CROSSWALK_RIGHT = 580, 675
GUIA = 42
ROBOT_SPEED = 156.0
TRIP_LIMIT = 90.0
PEDESTRIAN_MIN_TIME = (
    (ROAD_HEIGHT + 24 + GUIA) / ROBOT_SPEED * 1.35 + 0.4
)

WHITE = (245, 245, 245)
BLACK = (20, 22, 28)
ASPHALT = (42, 45, 52)
SIDEWALK = (130, 134, 142)
CURB = (175, 180, 188)
TACTILE = (235, 190, 20)
GRASS = (46, 115, 65)
ROBOT_BLUE = (30, 136, 229)
ROBOT_DARK = (15, 76, 129)
GREEN = (46, 204, 113)
ORANGE = (255, 140, 0)
RED = (225, 55, 55)
PURPLE = (155, 89, 182)


DESTINATIONS = {
    1: ("Mercado", (175, 540)),
    2: ("Farmácia", (285, 540)),
    3: ("Padaria", (470, 540)),
    4: ("Casa", (95, 127)),
}

SIGNAL_PHASES = (
    ("VERDE", 8.0),
    ("AMARELO", 2.0),
    ("TODOS_VERM_1", 1.5),
    ("PEDESTRE", 8.0),
    ("TODOS_VERM_2", 1.5),
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rotate_point(cx: float, cy: float, px: float, py: float, angle: float):
    sin_a, cos_a = math.sin(angle), math.cos(angle)
    dx, dy = px - cx, py - cy
    return int(cx + dx * cos_a - dy * sin_a), int(cy + dx * sin_a + dy * cos_a)


def make_static_obstacles():
    return [
        {"rect": pygame.Rect(230, 85, 45, 75), "type": "solo", "name": "Banca"},
        {"rect": pygame.Rect(460, 160, 32, 32), "type": "solo", "name": "Poste"},
        {"rect": pygame.Rect(380, 460, 40, 40), "type": "solo", "name": "Lixeira"},
        {"rect": pygame.Rect(350, 75, 85, 65), "type": "suspenso", "name": "Árvore"},
        {"rect": pygame.Rect(520, 485, 95, 65), "type": "suspenso", "name": "Toldo"},
    ]


def tactile_at(x: float, y: float) -> bool:
    return (
        40 <= x <= 635 and 112 <= y <= 142
        or 613 <= x <= 643 and 120 <= y <= 250
        or 613 <= x <= 643 and ROAD_BOTTOM <= y <= 550
        or 150 <= x <= 635 and 528 <= y <= 558
    )


def sidewalk_or_crosswalk(x: float, y: float) -> bool:
    if not (15 <= x <= MAP_WIDTH - 15 and 30 <= y <= HEIGHT - 30):
        return False
    if ROAD_TOP <= y <= ROAD_BOTTOM:
        return CROSSWALK_LEFT + 15 <= x <= CROSSWALK_RIGHT - 15
    return True


class AStarPlanner:
    def __init__(self, obstacles: list[dict]):
        self.blocked: set[tuple[int, int]] = set()
        margins = {"solo": 30, "edificacao": 22, "suspenso": 38}
        for obstacle in obstacles:
            margin = margins.get(obstacle["type"], 30)
            for cell in self._cells_for_rect(obstacle["rect"].inflate(margin * 2, margin * 2)):
                self.blocked.add(cell)

    @staticmethod
    def _cells_for_rect(rect: pygame.Rect):
        x0 = max(0, rect.left // CELL)
        x1 = min(MAP_WIDTH // CELL, (rect.right + CELL - 1) // CELL)
        y0 = max(0, rect.top // CELL)
        y1 = min(HEIGHT // CELL, (rect.bottom + CELL - 1) // CELL)
        for gx in range(x0, x1):
            for gy in range(y0, y1):
                yield gx, gy

    @staticmethod
    def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return 0.45 * (max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy))

    def _walkable(self, cell: tuple[int, int]) -> bool:
        x, y = (cell[0] + 0.5) * CELL, (cell[1] + 0.5) * CELL
        return sidewalk_or_crosswalk(x, y) and cell not in self.blocked

    def plan(self, start: tuple[float, float], goal: tuple[float, float]):
        start_cell = (int(start[0] // CELL), int(start[1] // CELL))
        goal_cell = (int(goal[0] // CELL), int(goal[1] // CELL))
        if not self._walkable(goal_cell):
            return []
        if not self._walkable(start_cell):
            start_cell = min(
                (
                    (x, y)
                    for x in range(MAP_WIDTH // CELL)
                    for y in range(HEIGHT // CELL)
                    if self._walkable((x, y))
                ),
                key=lambda cell: math.hypot(
                    (cell[0] + 0.5) * CELL - start[0],
                    (cell[1] + 0.5) * CELL - start[1],
                ),
                default=start_cell,
            )
        frontier = [(0.0, start_cell)]
        came_from: dict[tuple[int, int], Optional[tuple[int, int]]] = {start_cell: None}
        costs = {start_cell: 0.0}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_cell:
                break
            for neighbor, step_cost in self._neighbors(current):
                if not self._walkable(neighbor):
                    continue
                if neighbor[0] != current[0] and neighbor[1] != current[1]:
                    if not (
                        self._walkable((neighbor[0], current[1]))
                        and self._walkable((current[0], neighbor[1]))
                    ):
                        continue
                cx, cy = (neighbor[0] + 0.5) * CELL, (neighbor[1] + 0.5) * CELL
                step_cost *= 0.45 if tactile_at(cx, cy) else 1.0
                new_cost = costs[current] + step_cost
                if new_cost >= costs.get(neighbor, math.inf):
                    continue
                costs[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + self._heuristic(neighbor, goal_cell)
                heapq.heappush(frontier, (priority, neighbor))
        if goal_cell not in came_from:
            return []
        result = []
        current: Optional[tuple[int, int]] = goal_cell
        while current is not None:
            result.append(((current[0] + 0.5) * CELL, (current[1] + 0.5) * CELL))
            current = came_from[current]
        result.reverse()
        return result

    @staticmethod
    def _neighbors(cell: tuple[int, int]):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                yield (cell[0] + dx, cell[1] + dy), math.sqrt(2) if dx and dy else 1.0


@dataclass
class Tutor:
    x: float
    y: float
    speed: float = 190.0

    def update(self, robot: "Robot", dt: float):
        target_x = robot.x - math.cos(robot.angle) * GUIA
        target_y = robot.y - math.sin(robot.angle) * GUIA
        dx, dy = target_x - self.x, target_y - self.y
        gap = math.hypot(dx, dy)
        if gap:
            step = min(gap, self.speed * dt)
            self.x += dx / gap * step
            self.y += dy / gap * step


@dataclass
class Robot:
    x: float = 95.0
    y: float = 127.0
    angle: float = 0.0
    tutor: Tutor = field(default_factory=lambda: Tutor(53.0, 127.0))
    path: list[tuple[float, float]] = field(default_factory=list)
    path_index: int = 0
    path_history: list[deque] = field(default_factory=list)
    lidar: list[dict] = field(default_factory=list)
    state: str = "SEGUINDO_ROTA"
    alert_color: tuple[int, int, int] = GREEN
    alert_text: str = "Rota planejada; aguardando travessia segura."
    step_animation: float = 0.0
    speed: float = 0.0
    distance_travelled: float = 0.0

    def __post_init__(self):
        self.path_history = [deque(maxlen=3) for _ in range(19)]

    def read_lidar(self, obstacles: list[dict], vehicles: list["Vehicle"], rng: random.Random):
        readings = []
        first_angle = self.angle - math.radians(150) / 2
        for index in range(19):
            angle = first_angle + index * (math.radians(150) / 18)
            end = (
                int(self.x + math.cos(angle) * 165),
                int(self.y + math.sin(angle) * 165),
            )
            hit_distance, hit_type = 165.0, None
            ray_start = (int(self.x), int(self.y))
            for obstacle in obstacles:
                clipped = obstacle["rect"].clipline(ray_start, end)
                if clipped:
                    d = distance(ray_start, clipped[0])
                    if d < hit_distance:
                        hit_distance, hit_type = d, obstacle["type"]
            for vehicle in vehicles:
                clipped = vehicle.rect.clipline(ray_start, end)
                if clipped:
                    d = distance(ray_start, clipped[0])
                    if d < hit_distance:
                        hit_distance, hit_type = d, "veiculo"
            border = pygame.Rect(15, 15, MAP_WIDTH - 30, HEIGHT - 30)
            clipped = border.clipline(ray_start, end)
            if not clipped:
                hit_distance, hit_type = min(hit_distance, 15.0), "parede"
            elif hit_distance == 165.0:
                end_inside = border.collidepoint(end)
                if not end_inside:
                    hit_distance = min(hit_distance, distance(ray_start, clipped[1]))
                    hit_type = "parede"

            measured = 165.0 if rng.random() < 0.02 else clamp(
                hit_distance + rng.gauss(0, 1.5), 0.0, 165.0
            )
            self.path_history[index].append((measured, hit_type))
            sorted_values = sorted(sample[0] for sample in self.path_history[index])
            median = sorted_values[len(sorted_values) // 2]
            median_type = min(
                self.path_history[index], key=lambda sample: abs(sample[0] - median)
            )[1]
            readings.append({"angle": angle, "distance": median, "type": median_type})
        self.lidar = readings

    def update_towards(
        self,
        target: tuple[float, float],
        desired_speed: float,
        dt: float,
        movement_allowed: Optional[Callable[[float, float], bool]] = None,
    ):
        dx, dy = target[0] - self.x, target[1] - self.y
        desired_angle = math.atan2(dy, dx)
        difference = (desired_angle - self.angle + math.pi) % (2 * math.pi) - math.pi
        self.angle += clamp(difference, -2.8 * dt, 2.8 * dt)
        leash_gap = distance((self.x, self.y), (self.tutor.x, self.tutor.y))
        tension = max(0.0, leash_gap - GUIA)
        speed_factor = clamp(1.0 - tension / 10.0, 0.0, 1.0)
        turn_factor = max(0.25, 1.0 - abs(difference) / math.pi)
        target_distance = math.hypot(dx, dy)
        speed = min(desired_speed, max(24.0, target_distance * 2.0))
        speed *= turn_factor * speed_factor
        old_x, old_y = self.x, self.y
        next_x = self.x + math.cos(self.angle) * speed * dt
        next_y = self.y + math.sin(self.angle) * speed * dt
        if movement_allowed is None or movement_allowed(next_x, next_y):
            self.x, self.y = next_x, next_y
        else:
            speed = 0.0
        self.distance_travelled += distance((old_x, old_y), (self.x, self.y))
        self.speed = speed
        self.step_animation += speed * dt * 0.25
        self.tutor.update(self, dt)


@dataclass
class Vehicle:
    identifier: int
    x: float
    y: float
    direction: int
    max_speed: float
    speed: float = 0.0
    infractor: bool = False
    width: int = 48
    height: int = 25

    @property
    def rect(self) -> pygame.Rect:
        return pygame.Rect(round(self.x - self.width / 2), round(self.y - self.height / 2), self.width, self.height)

    def update(
        self,
        dt: float,
        signal_state: str,
        vehicles: list["Vehicle"],
        pedestrian_crossing: bool = False,
    ):
        stop_line = CROSSWALK_LEFT - 12 if self.direction > 0 else CROSSWALK_RIGHT + 12
        front = self.x + self.direction * self.width / 2
        distance_to_line = (stop_line - front) * self.direction
        red_for_cars = pedestrian_crossing or signal_state in (
            "TODOS_VERM_1", "PEDESTRE", "TODOS_VERM_2"
        )
        yellow = signal_state == "AMARELO"
        should_stop = red_for_cars and not self.infractor
        should_stop = should_stop or (
            yellow and not self.infractor and distance_to_line > 0
            and distance_to_line < self.speed * self.speed / (2 * 220.0) + 35
        )
        following_speed = self.max_speed
        for other in vehicles:
            if other is self or other.y != self.y or other.direction != self.direction:
                continue
            gap = (
                (other.x - self.x) * self.direction
                - (self.width + other.width) / 2
            )
            if gap > 0:
                following_speed = min(
                    following_speed,
                    other.speed + max(0.0, gap - 24.0) * 1.5,
                )
        desired = 0.0 if should_stop and distance_to_line >= 0 else following_speed
        acceleration = 220.0 if desired < self.speed else 105.0
        delta = clamp(desired - self.speed, -acceleration * dt, acceleration * dt)
        self.speed = max(0.0, self.speed + delta)
        self.x += self.direction * self.speed * dt
        if self.x < -70:
            self.x = MAP_WIDTH + 45
            self.speed = self.max_speed * 0.75
        elif self.x > MAP_WIDTH + 70:
            self.x = -45
            self.speed = self.max_speed * 0.75


class TrafficSignal:
    def __init__(self):
        self.phase_index = 0
        self.elapsed = 0.0

    @property
    def state(self) -> str:
        return SIGNAL_PHASES[self.phase_index][0]

    @property
    def remaining(self) -> float:
        return max(0.0, SIGNAL_PHASES[self.phase_index][1] - self.elapsed)

    def update(self, dt: float):
        self.elapsed += dt
        while self.elapsed >= SIGNAL_PHASES[self.phase_index][1]:
            self.elapsed -= SIGNAL_PHASES[self.phase_index][1]
            self.phase_index = (self.phase_index + 1) % len(SIGNAL_PHASES)


class StereoAlert:
    def __init__(self):
        self.enabled = False
        try:
            if not pygame.mixer.get_init():
                pygame.mixer.init(frequency=22050, size=-16, channels=2, buffer=512)
            self.enabled = True
        except pygame.error as error:
            print(f"Áudio indisponível: {error}", file=sys.stderr)

    def play(self, pan: float):
        if not self.enabled:
            return
        amplitude = 7000
        frequency = 880
        frames = bytearray()
        for index in range(2205):
            sample = int(amplitude * math.sin(2 * math.pi * frequency * index / 22050))
            fade = min(1.0, index / 100, (2205 - index) / 200)
            sample = int(sample * fade)
            left = int(sample * math.sqrt((1.0 - pan) / 2))
            right = int(sample * math.sqrt((1.0 + pan) / 2))
            frames.extend(left.to_bytes(2, "little", signed=True))
            frames.extend(right.to_bytes(2, "little", signed=True))
        pygame.mixer.Sound(buffer=bytes(frames)).play()


class Simulation:
    def __init__(self, seed: int = 1, destination: int = 1, audio: Optional[StereoAlert] = None):
        self.rng = random.Random(seed)
        self.obstacles = make_static_obstacles()
        self.planner = AStarPlanner(self.obstacles)
        self.robot = Robot()
        self.signal = TrafficSignal()
        self.vehicles = self._make_vehicles()
        self.audio = audio
        self.destination_id = destination
        self.destination_name, self.goal = DESTINATIONS[destination]
        self.elapsed = 0.0
        self.state = "SEGUINDO_ROTA"
        self.red_alerts = 0
        self.emergency_brakes = 0
        self.collisions = 0
        self.near_collisions = 0
        self.near_miss_vehicles: set[int] = set()
        self.emergency_active = False
        self.stuck_time = 0.0
        self.last_position = (self.robot.x, self.robot.y)
        self.last_alert_state = ""
        self.completed = False
        self.failed = False
        self._plan_path()
        self.robot.tutor.x = self.robot.x - GUIA

    def _make_vehicles(self):
        vehicles = []
        identifier = 0
        for lane_y, direction, positions in (
            (300, 1, (95, 350, 535)),
            (370, -1, (745, 465, 250)),
        ):
            for x in positions:
                infractor = self.rng.random() < 0.11
                max_speed = self.rng.uniform(62.0, 95.0)
                vehicles.append(Vehicle(identifier, float(x), lane_y, direction, max_speed,
                                        max_speed * 0.7, infractor))
                identifier += 1
        return vehicles

    def _plan_path(self):
        self.robot.path = self.planner.plan((self.robot.x, self.robot.y), self.goal)
        self.robot.path_index = 0
        if (
            self.robot.path
            and distance((self.robot.x, self.robot.y), self.robot.path[0]) <= 13
        ):
            self.robot.path_index = min(1, len(self.robot.path) - 1)

    def _road_entry(self, x: float, y: float) -> bool:
        return (
            CROSSWALK_LEFT + 15 <= x <= CROSSWALK_RIGHT - 15
            and ROAD_TOP - CELL <= y <= ROAD_BOTTOM + CELL
            and not ROAD_TOP <= self.robot.y <= ROAD_BOTTOM
        )

    def apply_barrier(self, x: float, y: float) -> bool:
        start_x, start_y = self.robot.x, self.robot.y
        start_in_road = ROAD_TOP <= start_y <= ROAD_BOTTOM
        end_in_road = ROAD_TOP <= y <= ROAD_BOTTOM
        if not start_in_road and not end_in_road:
            return True
        min_x, max_x = CROSSWALK_LEFT + 15, CROSSWALK_RIGHT - 15
        if start_in_road and end_in_road:
            if min_x <= x <= max_x:
                return True
            return (x < min_x and x > start_x) or (x > max_x and x < start_x)

        crossed_boundaries = []
        if start_y < ROAD_TOP <= y or y < ROAD_TOP <= start_y:
            crossed_boundaries.append(ROAD_TOP)
        if start_y <= ROAD_BOTTOM < y or y <= ROAD_BOTTOM < start_y:
            crossed_boundaries.append(ROAD_BOTTOM)
        for boundary in crossed_boundaries:
            ratio = (boundary - start_y) / (y - start_y)
            crossing_x = start_x + (x - start_x) * ratio
            if not min_x <= crossing_x <= max_x:
                return False

        if not start_in_road and end_in_road:
            return (
                self.signal.state == "PEDESTRE"
                and self.signal.remaining > PEDESTRIAN_MIN_TIME
            )
        return (
            start_in_road
            or (
                self.signal.state == "PEDESTRE"
                and self.signal.remaining > PEDESTRIAN_MIN_TIME
            )
        )

    @staticmethod
    def _actor_rect(x: float, y: float) -> pygame.Rect:
        return pygame.Rect(round(x - 11), round(y - 11), 22, 22)

    def _predict_vehicle_conflict(self) -> bool:
        entering_crossing = self._road_entry(*self._next_waypoint())
        if entering_crossing:
            crossing_clearance_time = (
                (ROAD_HEIGHT + 24 + GUIA) / (ROBOT_SPEED * 0.25) + 0.4
            )
            zone_left = CROSSWALK_LEFT - 24
            zone_right = CROSSWALK_RIGHT + 24
            for vehicle in self.vehicles:
                if not vehicle.infractor or not ROAD_TOP <= vehicle.y <= ROAD_BOTTOM:
                    continue
                if vehicle.direction > 0:
                    if vehicle.x < zone_left:
                        distance_to_zone = zone_left - vehicle.x
                    elif vehicle.x <= zone_right:
                        distance_to_zone = 0.0
                    else:
                        distance_to_zone = (
                            MAP_WIDTH + 70 - vehicle.x + zone_left + 45
                        )
                elif vehicle.x > zone_right:
                    distance_to_zone = vehicle.x - zone_right
                elif vehicle.x >= zone_left:
                    distance_to_zone = 0.0
                else:
                    distance_to_zone = (
                        vehicle.x + 70 + MAP_WIDTH + 45 - zone_right
                    )
                approach_speed = max(vehicle.speed, vehicle.max_speed * 0.5)
                if distance_to_zone / approach_speed <= crossing_clearance_time:
                    return True

        robot_next = (
            self.robot.x + math.cos(self.robot.angle) * self.robot.speed,
            self.robot.y + math.sin(self.robot.angle) * self.robot.speed,
        )
        target = self._next_waypoint()
        dx, dy = target[0] - self.robot.x, target[1] - self.robot.y
        route_distance = max(1.0, math.hypot(dx, dy))
        route_angle = math.atan2(dy, dx)
        route_step = min(ROBOT_SPEED, route_distance)
        robot_route_next = (
            self.robot.x + dx / route_distance * route_step,
            self.robot.y + dy / route_distance * route_step,
        )
        tutor_route_next = (
            robot_route_next[0] - math.cos(route_angle) * GUIA,
            robot_route_next[1] - math.sin(route_angle) * GUIA,
        )
        for vehicle in self.vehicles:
            rect_now = vehicle.rect
            future_x = vehicle.x + vehicle.direction * vehicle.speed
            rect_future = pygame.Rect(
                round(future_x - vehicle.width / 2),
                round(vehicle.y - vehicle.height / 2),
                vehicle.width,
                vehicle.height,
            )
            swept = rect_now.union(rect_future).inflate(8, 8)
            for point in (
                (self.robot.x, self.robot.y), robot_next,
                (self.robot.tutor.x, self.robot.tutor.y), robot_route_next,
                tutor_route_next,
            ):
                if swept.colliderect(self._actor_rect(*point)):
                    return True
        return False

    def _check_vehicle_distances(self):
        actors = (
            self._actor_rect(self.robot.x, self.robot.y),
            self._actor_rect(self.robot.tutor.x, self.robot.tutor.y),
        )
        for vehicle in self.vehicles:
            rect = vehicle.rect
            if any(rect.colliderect(actor) for actor in actors):
                self.collisions += 1
                self.failed = True
                return
            closest = min(
                math.hypot(
                    max(actor.left - rect.right, rect.left - actor.right, 0),
                    max(actor.top - rect.bottom, rect.top - actor.bottom, 0),
                )
                for actor in actors
            )
            if closest <= 18 and vehicle.identifier not in self.near_miss_vehicles:
                self.near_collisions += 1
                self.near_miss_vehicles.add(vehicle.identifier)

    def _next_waypoint(self):
        while self.robot.path_index < len(self.robot.path) - 1:
            if distance(
                (self.robot.x, self.robot.y), self.robot.path[self.robot.path_index]
            ) > 13:
                break
            next_waypoint = self.robot.path[self.robot.path_index + 1]
            if not self.apply_barrier(*next_waypoint):
                break
            self.robot.path_index += 1
        return self.robot.path[self.robot.path_index] if self.robot.path else self.goal

    def _update_state(self):
        previous_state = self.state
        if self.completed:
            self.state = "DESTINO_ALCANCADO"
        elif self.emergency_active:
            self.state = "FREIO_EMERGENCIA"
        elif self._road_entry(*self._next_waypoint()) and (
            self.signal.state != "PEDESTRE"
            or self.signal.remaining <= PEDESTRIAN_MIN_TIME
        ):
            self.state = "AGUARDANDO_SINAL"
        else:
            self.state = "SEGUINDO_ROTA"
        self.robot.state = self.state
        if self.state == "FREIO_EMERGENCIA":
            self.robot.alert_color = RED
            self.robot.alert_text = "FREIO DE EMERGÊNCIA: veículo na zona de risco!"
            if previous_state != "FREIO_EMERGENCIA":
                self.red_alerts += 1
        elif self.state == "AGUARDANDO_SINAL":
            self.robot.alert_color = ORANGE
            self.robot.alert_text = "Aguardando fase de pedestres segura."
        elif self.state == "DESTINO_ALCANCADO":
            self.robot.alert_color = GREEN
            self.robot.alert_text = "Destino alcançado com segurança."
        else:
            self.robot.alert_color = GREEN
            self.robot.alert_text = "Rota A*; seguindo piso tátil."

    def update(self, dt: float):
        if self.completed or self.failed:
            return
        self.elapsed += dt
        self.signal.update(dt)
        pedestrian_crossing = any(
            ROAD_TOP <= y <= ROAD_BOTTOM
            for y in (self.robot.y, self.robot.tutor.y)
        )
        for vehicle in self.vehicles:
            vehicle.update(dt, self.signal.state, self.vehicles, pedestrian_crossing)
        self.robot.read_lidar(self.obstacles, self.vehicles, self.rng)

        conflict = self._predict_vehicle_conflict()
        if conflict and not self.emergency_active:
            self.emergency_brakes += 1
        self.emergency_active = conflict
        self._check_vehicle_distances()
        if self.failed:
            return

        current_position = (self.robot.x, self.robot.y)
        waiting_for_signal = self._road_entry(*self._next_waypoint()) and (
            self.signal.state != "PEDESTRE"
            or self.signal.remaining <= PEDESTRIAN_MIN_TIME
        )
        if (
            distance(current_position, self.last_position) < 0.25
            and not self.emergency_active
            and not waiting_for_signal
        ):
            self.stuck_time += dt
        else:
            self.stuck_time = 0.0
        self.last_position = current_position
        if self.stuck_time > 3.5:
            self._plan_path()
            self.stuck_time = 0.0

        if self.audio and self.state != self.last_alert_state:
            center = self.robot.lidar[9]["distance"] if self.robot.lidar else 165
            if center < 65:
                side = -1.0 if self.robot.lidar[5]["distance"] < self.robot.lidar[13]["distance"] else 1.0
                self.audio.play(side)
        self.last_alert_state = self.state

        self._update_state()
        if self.state == "FREIO_EMERGENCIA":
            self.robot.speed = max(0.0, self.robot.speed - 420.0 * dt)
        elif self.state == "AGUARDANDO_SINAL":
            self.robot.speed = 0.0
        else:
            target = self._next_waypoint()
            self.robot.update_towards(
                target, ROBOT_SPEED, dt, movement_allowed=self.apply_barrier
            )

        self._check_vehicle_distances()
        if self.failed:
            return
        if distance((self.robot.x, self.robot.y), self.goal) <= 16:
            self.completed = True
            self.state = "DESTINO_ALCANCADO"
        if self.elapsed >= TRIP_LIMIT and not self.completed:
            self.failed = True

    def metrics(self):
        return {
            "completed": self.completed,
            "time_seconds": round(self.elapsed, 3),
            "distance_px": round(self.robot.distance_travelled, 1),
            "red_alerts": self.red_alerts,
            "emergency_brakes": self.emergency_brakes,
            "collisions": self.collisions,
            "near_collisions": self.near_collisions,
        }


def run_benchmark(trips: int, seed: int):
    results = []
    for index in range(trips):
        simulation = Simulation(seed=seed + index)
        steps = int(TRIP_LIMIT * FPS)
        for _ in range(steps):
            simulation.update(1.0 / FPS)
            if simulation.completed or simulation.failed:
                break
        results.append(simulation.metrics())
    completed = sum(result["completed"] for result in results)
    return {
        "seed": seed,
        "trips": trips,
        "success_rate_percent": round(100 * completed / trips, 1) if trips else 0,
        "average_time_seconds": round(
            sum(result["time_seconds"] for result in results) / trips, 2
        ) if trips else 0,
        "collisions": sum(result["collisions"] for result in results),
        "near_collisions": sum(result["near_collisions"] for result in results),
        "emergency_brakes": sum(result["emergency_brakes"] for result in results),
        "completed_trips": completed,
        "per_trip": results,
    }


def draw_environment(screen):
    screen.fill(SIDEWALK)
    pygame.draw.rect(screen, GRASS, (0, 0, MAP_WIDTH, 28))
    pygame.draw.rect(screen, GRASS, (0, HEIGHT - 28, MAP_WIDTH, 28))
    pygame.draw.rect(screen, ASPHALT, (0, ROAD_TOP, MAP_WIDTH, ROAD_HEIGHT))
    pygame.draw.line(screen, CURB, (0, ROAD_TOP), (MAP_WIDTH, ROAD_TOP), 4)
    pygame.draw.line(screen, CURB, (0, ROAD_BOTTOM), (MAP_WIDTH, ROAD_BOTTOM), 4)
    for x in range(15, MAP_WIDTH - 20, 45):
        if not CROSSWALK_LEFT - 10 <= x <= CROSSWALK_RIGHT + 10:
            pygame.draw.line(screen, WHITE, (x, ROAD_TOP + ROAD_HEIGHT // 2),
                             (x + 22, ROAD_TOP + ROAD_HEIGHT // 2), 3)
    for y in range(ROAD_TOP + 12, ROAD_BOTTOM - 12, 22):
        pygame.draw.rect(screen, WHITE, (CROSSWALK_LEFT, y, CROSSWALK_RIGHT - CROSSWALK_LEFT, 12))
    pygame.draw.rect(screen, TACTILE, (40, 120, 595, 14))
    pygame.draw.rect(screen, TACTILE, (621, 120, 14, 130))
    pygame.draw.rect(screen, TACTILE, (621, ROAD_BOTTOM, 14, 130))
    pygame.draw.rect(screen, TACTILE, (150, 536, 485, 14))


def draw_robot(screen, robot: Robot):
    for ray in robot.lidar:
        end = (
            robot.x + math.cos(ray["angle"]) * ray["distance"],
            robot.y + math.sin(ray["angle"]) * ray["distance"],
        )
        color = GREEN if ray["distance"] >= 160 else RED
        pygame.draw.line(screen, color, (robot.x, robot.y), end, 1)
    user = (int(robot.tutor.x), int(robot.tutor.y))
    pygame.draw.line(screen, (255, 215, 0), (robot.x, robot.y), user, 4)
    pygame.draw.circle(screen, WHITE, user, 12)
    pygame.draw.circle(screen, BLACK, user, 12, 2)
    sway = math.sin(robot.step_animation) * 5
    for ox, oy in ((10 + sway, -14), (10 - sway, 14), (-12 - sway, -14), (-12 + sway, 14)):
        pygame.draw.circle(screen, ROBOT_DARK,
                           rotate_point(robot.x, robot.y, robot.x + ox, robot.y + oy, robot.angle), 5)
    body = [
        rotate_point(robot.x, robot.y, robot.x + 16, robot.y - 11, robot.angle),
        rotate_point(robot.x, robot.y, robot.x + 16, robot.y + 11, robot.angle),
        rotate_point(robot.x, robot.y, robot.x - 18, robot.y + 11, robot.angle),
        rotate_point(robot.x, robot.y, robot.x - 18, robot.y - 11, robot.angle),
    ]
    pygame.draw.polygon(screen, ROBOT_BLUE, body)
    pygame.draw.polygon(screen, WHITE, body, 2)
    head = rotate_point(robot.x, robot.y, robot.x + 21, robot.y, robot.angle)
    pygame.draw.circle(screen, WHITE, head, 9)
    pygame.draw.circle(screen, ORANGE, rotate_point(robot.x, robot.y, robot.x + 3, robot.y, robot.angle), 5)


def draw_ui(screen, simulation: Simulation, font, small_font):
    panel_x = MAP_WIDTH
    pygame.draw.rect(screen, BLACK, (panel_x, 0, WIDTH - panel_x, HEIGHT))
    pygame.draw.line(screen, WHITE, (panel_x, 0), (panel_x, HEIGHT), 2)
    lines = [
        ("DOG-IA V2", ROBOT_BLUE),
        (f"Estado: {simulation.state}", WHITE),
        (f"Semáforo: {simulation.signal.state}", WHITE),
        (f"Destino: {simulation.destination_name}", WHITE),
        (f"LiDAR: {min((r['distance'] for r in simulation.robot.lidar), default=165):.0f} px", WHITE),
        (f"Tempo: {simulation.elapsed:.1f} s", WHITE),
        (f"Freios: {simulation.emergency_brakes}", WHITE),
        (simulation.robot.alert_text, simulation.robot.alert_color),
    ]
    y = 18
    for index, (text, color) in enumerate(lines):
        selected_font = font if index in (0, 7) else small_font
        for line in wrap_text(text, selected_font, WIDTH - panel_x - 28):
            screen.blit(selected_font.render(line, True, color), (panel_x + 14, y))
            y += 24 if selected_font == font else 22
        y += 7
    y = 330
    for text in (
        "CONTROLES",
        "1-4: Mercado/Farm./Pad./Casa",
        "Clique: escolher destino",
        "Clique direito: obstáculo",
        "R: reiniciar  |  ESC: sair",
        "",
        "Veículos e semáforo ativos",
        "Verde: trânsito | Amarelo",
        "All-Red | Pedestre | All-Red",
    ):
        screen.blit(small_font.render(text, True, WHITE), (panel_x + 14, y))
        y += 27


def wrap_text(text, font, max_width):
    words = text.split()
    lines, current = [], ""
    for word in words:
        candidate = f"{current} {word}".strip()
        if current and font.size(candidate)[0] > max_width:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    return lines


def run_window(seed: int):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("DOG-IA V2 — Cão-Guia Autônomo")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("arial", 18, bold=True)
    small_font = pygame.font.SysFont("arial", 14)
    audio = StereoAlert()
    simulation = Simulation(seed=seed, audio=audio)
    running = True
    while running:
        dt = min(clock.tick(FPS) / 1000.0, 0.05)
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
            ):
                running = False
            elif event.type == pygame.KEYDOWN and event.key in (
                pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4
            ):
                selected = event.key - pygame.K_0
                simulation.destination_id = selected
                simulation.destination_name, simulation.goal = DESTINATIONS[selected]
                simulation.completed = False
                simulation.failed = False
                simulation._plan_path()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                simulation = Simulation(seed=seed, audio=audio)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.pos[0] < MAP_WIDTH:
                if event.button == 1:
                    simulation.goal = event.pos
                    simulation.destination_name = "Personalizado"
                    simulation.completed = False
                    simulation._plan_path()
                elif event.button == 3:
                    kind = simulation.rng.choice(("solo", "suspenso"))
                    simulation.obstacles.append({
                        "rect": pygame.Rect(event.pos[0] - 22, event.pos[1] - 22, 44, 44),
                        "type": kind, "name": "Obstáculo",
                    })
                    simulation.planner = AStarPlanner(simulation.obstacles)
                    simulation._plan_path()

        simulation.update(dt)
        draw_environment(screen)
        pygame.draw.circle(screen, GREEN, (int(simulation.goal[0]), int(simulation.goal[1])), 17, 3)
        for obstacle in simulation.obstacles:
            color = PURPLE if obstacle["type"] == "suspenso" else (70, 74, 82)
            pygame.draw.rect(screen, color, obstacle["rect"], border_radius=6)
            pygame.draw.rect(screen, WHITE, obstacle["rect"], 2, border_radius=6)
        for vehicle in simulation.vehicles:
            pygame.draw.rect(screen, (205, 70, 60) if vehicle.infractor else (210, 210, 220),
                             vehicle.rect, border_radius=5)
            pygame.draw.rect(screen, BLACK, vehicle.rect, 2, border_radius=5)
        draw_robot(screen, simulation.robot)
        draw_ui(screen, simulation, font, small_font)
        pygame.display.flip()
    pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="Simulador urbano DOG-IA V2.")
    parser.add_argument("--benchmark", type=int, metavar="N",
                        help="executa N viagens sem abrir janela")
    parser.add_argument("--seed", type=int, default=1, help="semente aleatória")
    args = parser.parse_args()
    if args.benchmark is not None:
        if args.benchmark < 1:
            parser.error("--benchmark deve ser maior que zero")
        pygame.init()
        pygame.display.quit()
        report = run_benchmark(args.benchmark, args.seed)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        pygame.quit()
        return
    run_window(args.seed)


if __name__ == "__main__":
    main()
