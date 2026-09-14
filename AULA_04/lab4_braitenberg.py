"""Exercicio 4: Braitenberg com conexoes diretas (atracao/agressao)."""
import argparse
import math


def wheel_speeds(v0, alpha, left_distance, right_distance, max_distance):
    left = v0 + alpha * (1.0 - left_distance / max_distance)
    right = v0 + alpha * (1.0 - right_distance / max_distance)
    return left, right


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    left, right = wheel_speeds(35.0, 45.0, 160.0, 35.0, 200.0)
    print("Obstaculo a direita: vL={:.2f} px/s, vR={:.2f} px/s".format(left, right))
    print("Como vR > vL, o robo curva para a direita: comportamento de atracao/agressao.")
    if args.headless:
        return
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((900, 560))
    clock = pygame.time.Clock()
    robot = [140.0, 280.0, 0.0]
    obstacle = pygame.Rect(680, 190, 100, 180)
    trail = []
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
        dx, dy = obstacle.centerx - robot[0], obstacle.centery - robot[1]
        distance = max(1.0, math.hypot(dx, dy))
        angle_to_obstacle = math.atan2(dy, dx)
        left_distance = min(200.0, max(10.0, distance + math.sin(angle_to_obstacle - robot[2]) * 90))
        right_distance = min(200.0, max(10.0, distance - math.sin(angle_to_obstacle - robot[2]) * 90))
        left, right = wheel_speeds(35.0, 45.0, left_distance, right_distance, 200.0)
        linear = (left + right) / 2
        angular = (right - left) / 70.0
        dt = 1.0 / 60.0
        robot[2] += angular * dt
        robot[0] += linear * math.cos(robot[2]) * dt
        robot[1] += linear * math.sin(robot[2]) * dt
        robot[0] = max(30, min(870, robot[0])); robot[1] = max(30, min(530, robot[1]))
        trail.append((round(robot[0]), round(robot[1]))); trail = trail[-1500:]
        screen.fill((31, 35, 43)); pygame.draw.rect(screen, (180, 70, 55), obstacle)
        if len(trail) > 1: pygame.draw.lines(screen, (80, 190, 220), False, trail, 2)
        pygame.draw.circle(screen, (235, 190, 65), (round(robot[0]), round(robot[1])), 18)
        pygame.display.flip(); clock.tick(60)
    pygame.quit()


if __name__ == "__main__":
    main()
