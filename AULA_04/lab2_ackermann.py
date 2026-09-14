"""Exercicio 2: raio e velocidade angular de um veiculo Ackermann."""
import argparse
import math

WHEELBASE = 2.0
MAX_STEERING = math.radians(30.0)


def ackermann_values(linear_velocity, steering_angle):
    steering_angle = max(-MAX_STEERING, min(MAX_STEERING, steering_angle))
    angular_velocity = linear_velocity / WHEELBASE * math.tan(steering_angle)
    if abs(steering_angle) < 1e-9:
        radius = math.inf
    else:
        radius = WHEELBASE / abs(math.tan(steering_angle))
    return steering_angle, angular_velocity, radius


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--angle", type=float, default=20.0, help="angulo em graus")
    args = parser.parse_args()
    angle, angular_velocity, radius = ackermann_values(args.velocity, math.radians(args.angle))
    print("Ackermann: v={:.2f} m/s, phi={:.2f} deg, omega={:.4f} rad/s, R={}".format(args.velocity, math.degrees(angle), angular_velocity, "infinito" if math.isinf(radius) else "{:.4f} m".format(radius)))
    print("Limite aplicado: phi entre -30.00 e +30.00 graus")
    if args.headless:
        return
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((900, 600))
    clock = pygame.time.Clock()
    velocity = args.velocity
    steering = angle
    position = [450.0, 450.0]
    heading = -math.pi / 2
    trail = []
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]: velocity += 0.01
        if keys[pygame.K_DOWN]: velocity = max(0.0, velocity - 0.01)
        if keys[pygame.K_LEFT]: steering = min(MAX_STEERING, steering + 0.01)
        if keys[pygame.K_RIGHT]: steering = max(-MAX_STEERING, steering - 0.01)
        _, angular, _ = ackermann_values(velocity, steering)
        dt = 1.0 / 60.0
        position[0] += velocity * math.cos(heading) * 40 * dt
        position[1] += velocity * math.sin(heading) * 40 * dt
        heading += angular * dt
        trail.append(tuple(position))
        trail = trail[-1500:]
        screen.fill((245, 241, 230))
        if len(trail) > 1: pygame.draw.lines(screen, (35, 110, 150), False, trail, 3)
        pygame.draw.circle(screen, (210, 65, 55), (round(position[0]), round(position[1])), 14)
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()


if __name__ == "__main__":
    main()
