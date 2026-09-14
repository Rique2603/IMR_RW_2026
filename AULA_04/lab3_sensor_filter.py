"""Exercicio 3: varredura de sete feixes com ruido e threshold."""
import argparse
import math
import numpy as np

BEAM_ANGLES = np.linspace(-math.pi / 2, math.pi / 2, 7)
MIN_RANGE = 10.0
MAX_RANGE = 200.0


def apply_threshold(readings):
    readings = np.asarray(readings, dtype=float)
    return np.clip(readings, MIN_RANGE, MAX_RANGE)


def scan(real_distances, seed=None):
    random = np.random.default_rng(seed)
    raw = np.asarray(real_distances, dtype=float) + random.normal(0.0, 5.0, 7)
    return raw, apply_threshold(raw)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seed", type=int, default=4)
    args = parser.parse_args()
    real = np.array([35, 75, 140, 180, 140, 75, 35], dtype=float)
    raw, filtered = scan(real, args.seed)
    for index, angle in enumerate(BEAM_ANGLES):
        print("Feixe {:d} ({:6.1f} deg): bruto={:7.2f} px | tratado={:7.2f} px".format(index + 1, math.degrees(angle), raw[index], filtered[index]))
    if args.headless:
        return
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((900, 560))
    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
        screen.fill((24, 30, 38))
        center = np.array([450.0, 470.0])
        for index, angle in enumerate(BEAM_ANGLES):
            direction = np.array([math.cos(angle), -math.sin(angle)])
            start = center + direction * 20
            raw_end = center + direction * min(abs(raw[index]), MAX_RANGE) * 2
            filtered_end = center + direction * filtered[index] * 2
            pygame.draw.line(screen, (200, 80, 80), start.astype(int), raw_end.astype(int), 2)
            pygame.draw.line(screen, (70, 210, 150), start.astype(int), filtered_end.astype(int), 5)
            pygame.draw.circle(screen, (240, 240, 240), filtered_end.astype(int), 5)
        pygame.draw.circle(screen, (230, 180, 60), center.astype(int), 16)
        pygame.display.flip()
        clock.tick(30)
    pygame.quit()


if __name__ == "__main__":
    main()
