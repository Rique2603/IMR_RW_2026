"""Exercicio 1: pose de um robo diferencial em malha aberta."""
import argparse
import math


def integrate_pose(pose, linear_velocity, angular_velocity, duration):
    x, y, theta = pose
    if abs(angular_velocity) < 1e-12:
        x += linear_velocity * duration * math.cos(theta)
        y += linear_velocity * duration * math.sin(theta)
    else:
        radius = linear_velocity / angular_velocity
        x += radius * (math.sin(theta + angular_velocity * duration) - math.sin(theta))
        y += radius * (-math.cos(theta + angular_velocity * duration) + math.cos(theta))
    theta += angular_velocity * duration
    return x, y, theta


def theoretical_pose():
    pose = (0.0, 0.0, 0.0)
    for velocity, angular_velocity, duration in (
        (0.5, 0.0, 4.0),
        (0.0, 0.7854, 2.0),
        (0.4, 0.0, 3.0),
    ):
        pose = integrate_pose(pose, velocity, angular_velocity, duration)
    return pose


def run_simulation(pose, dt=0.01):
    simulated = pose
    for velocity, angular_velocity, duration in (
        (0.5, 0.0, 4.0),
        (0.0, 0.7854, 2.0),
        (0.4, 0.0, 3.0),
    ):
        steps = int(duration / dt)
        for _ in range(steps):
            simulated = integrate_pose(simulated, velocity, angular_velocity, dt)
    return simulated


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--screenshot", action="store_true")
    args = parser.parse_args()
    theoretical = theoretical_pose()
    simulated = run_simulation((0.0, 0.0, 0.0))
    print("Pose final teorica: x={:.4f} m, y={:.4f} m, theta={:.4f} rad ({:.2f} deg)".format(theoretical[0], theoretical[1], theoretical[2], math.degrees(theoretical[2])))
    print("Pose simulada:      x={:.4f} m, y={:.4f} m, theta={:.4f} rad ({:.2f} deg)".format(simulated[0], simulated[1], simulated[2], math.degrees(simulated[2])))
    if args.headless:
        return
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((800, 500))
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 28)
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill((20, 25, 35))
        scale = 70
        origin = (400, 250)
        first = (origin[0] + 2 * scale, origin[1])
        final = (origin[0] + theoretical[0] * scale, origin[1] - theoretical[1] * scale)
        pygame.draw.line(screen, (70, 180, 255), origin, first, 4)
        pygame.draw.line(screen, (70, 220, 150), first, final, 4)
        pygame.draw.circle(screen, (245, 190, 70), origin, 8)
        pygame.draw.circle(screen, (245, 190, 70), final, 12)
        label = font.render("Pose final: (2.00 m, 1.20 m, 90 deg)", True, (240, 240, 240))
        screen.blit(label, (20, 20))
        pygame.display.flip()
        if args.screenshot:
            pygame.image.save(screen, "AULA_04/exercicio1_pose.png")
            running = False
        clock.tick(60)
    pygame.quit()


if __name__ == "__main__":
    main()
