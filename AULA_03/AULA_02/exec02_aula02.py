import pygame
import math
import numpy as np

# Constantes de Configuração
LARGURA_TELA = 800
ALTURA_TELA = 600
FPS = 60
COR_FUNDO = (30, 30, 30)
COR_ROBO = (0, 180, 255)
COR_DIRECAO = (255, 50, 50)
COR_TRAJETORIA = (100, 200, 100)

class DiffDriveRobot:
    def __init__(self, x, y, theta=0.0, wheelbase=30.0, radius=15.0):
        # Estado do robô: [x, y, theta]
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)  # em radianos
        
        # Parâmetros físicos (em pixels)
        self.L = float(wheelbase)  # Distância entre rodas
        self.radius = float(radius)
        
        # Entradas de controle
        self.v = 0.0      # Velocidade linear (pixels/s)
        self.omega = 0.0  # Velocidade angular (rad/s)
        
        # Histórico de posições para plotar rastro
        self.history = []

    def set_wheel_velocities(self, v_left, v_right):
        """Converte velocidade das rodas em velocidade linear e angular."""
        self.v = (v_right + v_left) / 2.0
        self.omega = (v_right - v_left) / self.L

    def set_direct_velocity(self, v, omega):
        """Comando direto de velocidade linear e angular (padrão cmd_vel)."""
        self.v = v
        self.omega = omega

    def update(self, dt):
        """Integração numérica da cinemática diferencial."""
        # Atualização angular
        self.theta += self.omega * dt
        # Normaliza o ângulo entre [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
        # Atualização de posição cartesiana
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        
        # Guarda histórico para desenhar o rastro
        if len(self.history) == 0 or np.hypot(self.x - self.history[-1][0], self.y - self.history[-1][1]) > 5:
            self.history.append((self.x, self.y))
            if len(self.history) > 500:
                self.history.pop(0)

    def draw(self, surface):
        # 1. Desenha o rastro
        if len(self.history) > 1:
            pygame.draw.lines(surface, COR_TRAJETORIA, False, self.history, 2)
            
        # 2. Desenha o corpo do robô
        pos_int = (int(self.x), int(self.y))
        pygame.draw.circle(surface, COR_ROBO, pos_int, int(self.radius))
        
        # 3. Desenha a linha indicadora da direção (orientação theta)
        linha_frente_x = self.x + (self.radius + 10) * math.cos(self.theta)
        linha_frente_y = self.y + (self.radius + 10) * math.sin(self.theta)
        pygame.draw.line(surface, COR_DIRECAO, pos_int, (int(linha_frente_x), int(linha_frente_y)), 3)

def main():
    pygame.init()
    screen = pygame.display.set_mode((LARGURA_TELA, ALTURA_TELA))
    pygame.display.set_caption("Aula 02: Quadrado em Malha Aberta")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 14)

    robot = DiffDriveRobot(x=LARGURA_TELA // 2, y=ALTURA_TELA // 2, theta=0.0)

    # Máquina de estados do exercício 2
    state = "STRAIGHT"
    state_timer = 0.0
    side_count = 0
    max_sides = 4
    straight_time = 2.0
    turn_time = 1.0
    linear_speed = 100.0
    angular_speed = math.pi / 2.0

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        state_timer += dt

        if state == "STRAIGHT":
            robot.set_direct_velocity(linear_speed, 0.0)
            if state_timer >= straight_time:
                state = "TURN"
                state_timer = 0.0
                robot.set_direct_velocity(0.0, angular_speed)

        elif state == "TURN":
            robot.set_direct_velocity(0.0, angular_speed)
            if state_timer >= turn_time:
                side_count += 1
                if side_count >= max_sides:
                    state = "DONE"
                    robot.set_direct_velocity(0.0, 0.0)
                else:
                    state = "STRAIGHT"
                    state_timer = 0.0
                    robot.set_direct_velocity(linear_speed, 0.0)

        elif state == "DONE":
            robot.set_direct_velocity(0.0, 0.0)

        robot.update(dt)

        screen.fill(COR_FUNDO)
        robot.draw(screen)

        state_label = {
            "STRAIGHT": "Reta",
            "TURN": "Giro",
            "DONE": "Concluído"
        }

        info_txt = [
            f"Pose X: {robot.x:.1f} px | Y: {robot.y:.1f} px | Theta: {math.degrees(robot.theta):.1f} deg",
            f"Estado: {state_label.get(state, state)} | v = {robot.v:.1f} px/s | omega = {robot.omega:.2f} rad/s",
            f"Lado: {side_count}/{max_sides} | Tempo do estado: {state_timer:.2f}s",
            "Exercício 2: quadrado em malha aberta (100 px/s por 2s e giro de 90° por 1s)"
        ]
        for i, txt in enumerate(info_txt):
            rendered = font.render(txt, True, (220, 220, 220))
            screen.blit(rendered, (15, 15 + i * 20))

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
