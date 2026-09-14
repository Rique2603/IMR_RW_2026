"""Exercicio 5: centralizacao em corredor com controlador proporcional."""
import argparse

CORRIDOR_LEFT = 100.0
CORRIDOR_RIGHT = 500.0
KP = 0.01
LINEAR_SPEED = 40.0


def proportional_control(left_distance, right_distance, kp=KP):
    error = left_distance - right_distance
    return error, kp * error


def simulate(initial_y=170.0, duration=10.0, dt=0.02):
    y = initial_y
    heading = 0.0
    samples = []
    for _ in range(int(duration / dt)):
        left_distance = y - CORRIDOR_LEFT
        right_distance = CORRIDOR_RIGHT - y
        error, angular_velocity = proportional_control(left_distance, right_distance)
        y += LINEAR_SPEED * __import__("math").sin(heading) * dt
        heading -= angular_velocity * dt
        heading *= 0.96
        samples.append((y, error, heading))
    return samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    samples = simulate()
    print("Controlador: e = d_esq - d_dir; omega = Kp * e; Kp={:.3f}; v={:.1f} px/s".format(KP, LINEAR_SPEED))
    print("Estado inicial: y=170.00 px | estado final: y={:.2f} px | erro final={:.2f} px".format(samples[-1][0], samples[-1][1]))
    if args.headless:
        return
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((900, 600))
    clock = pygame.time.Clock()
    y = 170.0; x = 80.0; heading = 0.0
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
        left_distance = y - CORRIDOR_LEFT; right_distance = CORRIDOR_RIGHT - y
        _, angular = proportional_control(left_distance, right_distance)
        dt = 1.0 / 60.0
        x += LINEAR_SPEED * __import__("math").cos(heading) * dt
        y += LINEAR_SPEED * __import__("math").sin(heading) * dt
        heading -= angular * dt
        heading *= 0.96
        if x > 850: x = 80
        y = max(CORRIDOR_LEFT + 20, min(CORRIDOR_RIGHT - 20, y))
        screen.fill((238, 234, 220)); pygame.draw.rect(screen, (75, 75, 80), (0, 100, 900, 10)); pygame.draw.rect(screen, (75, 75, 80), (0, 500, 900, 10))
        pygame.draw.circle(screen, (45, 125, 155), (round(x), round(y)), 18)
        pygame.display.flip(); clock.tick(60)
    pygame.quit()


if __name__ == "__main__":
    main()
