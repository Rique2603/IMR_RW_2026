#!/usr/bin/env python3
"""
LAB-1 AULA-03: Sistema de Raycasting para Robô em PyGame
Implementação completa de um robô virtual com sensores de detecção de obstáculos

Autor: Gerado pela atividade
Data: 2026
"""

import pygame
import math
import numpy as np

# Constantes de configuração
LARGURA, ALTURA = 900, 650
FPS = 60
COR_FUNDO = (20, 24, 30)
COR_ROBO = (0, 200, 255)
COR_OBSTACULO = (180, 50, 50)
COR_RAIO_LIVRE = (0, 255, 100)
COR_RAIO_COLISAO = (255, 200, 0)

class RaycastDemoRobot:
    """
    Classe que representa um robô com capacidade de raycasting.
    O robô possui 3 sensores em ângulos diferentes para detectar obstáculos.
    """
    
    def __init__(self, x, y, theta=0.0):
        """
        Inicializa o robô.
        
        Args:
            x (float): Posição X inicial
            y (float): Posição Y inicial
            theta (float): Orientação inicial (radianos)
        """
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.sensor_angles = [-math.pi / 4, 0.0, math.pi / 4]  # Esq, Frente, Dir
        self.sensor_range = 150.0
        self.sensor_readings = [self.sensor_range] * 3

    def cast_rays(self, obstacles):
        """
        Verifica a interseção dos raios com obstáculos retangulares.
        Calcula a distância de cada sensor até o obstáculo mais próximo.
        
        Args:
            obstacles (list): Lista de retângulos (x, y, w, h)
        """
        self.sensor_readings = []
        for beta in self.sensor_angles:
            angle = self.theta + beta
            # Calcula a direção do raio
            dx = math.cos(angle)
            dy = math.sin(angle)
            
            min_distance = self.sensor_range
            
            # Verifica colisão com cada obstáculo
            for obs in obstacles:
                distance = self._ray_rect_intersection(self.x, self.y, dx, dy, obs)
                if distance is not None and distance < min_distance:
                    min_distance = distance
            
            self.sensor_readings.append(min_distance)

    def _ray_rect_intersection(self, x0, y0, dx, dy, rect):
        """
        Calcula interseção de um raio com um retângulo usando algoritmo AABB.
        
        Args:
            x0, y0 (float): Origem do raio
            dx, dy (float): Direção do raio (normalizada)
            rect (tuple): Retângulo (x, y, w, h)
            
        Returns:
            float: Distância da interseção ou None se não houver colisão
        """
        rx, ry, rw, rh = rect
        
        # Limites do retângulo
        x_min, x_max = rx, rx + rw
        y_min, y_max = ry, ry + rh
        
        t_min = float('-inf')
        t_max = float('inf')
        
        # Verificação em X
        if abs(dx) > 1e-6:
            t1 = (x_min - x0) / dx
            t2 = (x_max - x0) / dx
            if t1 > t2:
                t1, t2 = t2, t1
            t_min = max(t_min, t1)
            t_max = min(t_max, t2)
        elif x0 < x_min or x0 > x_max:
            return None
        
        # Verificação em Y
        if abs(dy) > 1e-6:
            t1 = (y_min - y0) / dy
            t2 = (y_max - y0) / dy
            if t1 > t2:
                t1, t2 = t2, t1
            t_min = max(t_min, t1)
            t_max = min(t_max, t2)
        elif y0 < y_min or y0 > y_max:
            return None
        
        # Se há interseção e t > 0 (raio vai para frente)
        if t_min < t_max and t_min > 0 and t_min < self.sensor_range:
            return t_min
        
        return None

    def draw(self, screen):
        """
        Desenha o robô e seus sensores na tela.
        
        Args:
            screen: Surface do pygame para desenhar
        """
        # Desenha o corpo do robô
        pygame.draw.circle(screen, COR_ROBO, (int(self.x), int(self.y)), 8)
        
        # Desenha a orientação (linha na direção theta)
        end_x = self.x + 12 * math.cos(self.theta)
        end_y = self.y + 12 * math.sin(self.theta)
        pygame.draw.line(screen, COR_ROBO, (self.x, self.y), (end_x, end_y), 2)
        
        # Desenha os raios dos sensores
        for i, beta in enumerate(self.sensor_angles):
            angle = self.theta + beta
            distance = self.sensor_readings[i]
            
            # Calcula ponto final do raio
            end_x = self.x + distance * math.cos(angle)
            end_y = self.y + distance * math.sin(angle)
            
            # Define cor baseada se houve colisão
            if distance < self.sensor_range - 0.1:  # houve colisão
                cor = COR_RAIO_COLISAO
            else:  # raio livre
                cor = COR_RAIO_LIVRE
            
            pygame.draw.line(screen, cor, (self.x, self.y), (end_x, end_y), 1)

    def update(self, keys, obstacles):
        """
        Atualiza a posição e orientação do robô baseado na entrada de teclado.
        
        Args:
            keys: Dicionário de teclas pressionadas do pygame
            obstacles: Lista de obstáculos para detecção de sensores
        """
        # Movimento: W/A/S/D ou Setas
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            self.x += 3 * math.cos(self.theta)
            self.y += 3 * math.sin(self.theta)
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:
            self.x -= 2 * math.cos(self.theta)
            self.y -= 2 * math.sin(self.theta)
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            self.theta -= 0.05
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            self.theta += 0.05
        
        # Limita posição dentro da tela
        self.x = max(0, min(LARGURA, self.x))
        self.y = max(0, min(ALTURA, self.y))
        
        # Atualiza leitura dos sensores
        self.cast_rays(obstacles)


def draw_obstacles(screen, obstacles):
    """
    Desenha os obstáculos na tela.
    
    Args:
        screen: Surface do pygame para desenhar
        obstacles: Lista de retângulos (x, y, w, h)
    """
    for obs in obstacles:
        x, y, w, h = obs
        pygame.draw.rect(screen, COR_OBSTACULO, (x, y, w, h))


def main():
    """Função principal que executa a simulação."""
    pygame.init()
    screen = pygame.display.set_mode((LARGURA, ALTURA))
    pygame.display.set_caption("Raycasting Demo Robot - LAB-1 AULA-03")
    clock = pygame.time.Clock()
    
    # Define obstáculos (x, y, largura, altura)
    obstacles = [
        (100, 100, 150, 30),
        (400, 150, 30, 200),
        (650, 400, 150, 30),
        (300, 450, 250, 30),
        (50, 350, 30, 150),
    ]
    
    # Cria o robô
    robot = RaycastDemoRobot(LARGURA // 2, ALTURA // 2)
    
    # Loop principal
    running = True
    while running:
        clock.tick(FPS)
        
        # Eventos
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        
        # Atualiza
        keys = pygame.key.get_pressed()
        robot.update(keys, obstacles)
        
        # Desenha
        screen.fill(COR_FUNDO)
        draw_obstacles(screen, obstacles)
        robot.draw(screen)
        
        # Exibe informações
        font = pygame.font.Font(None, 24)
        text = font.render(
            f"X: {robot.x:.1f} Y: {robot.y:.1f} Theta: {math.degrees(robot.theta):.1f}°",
            True, (255, 255, 255)
        )
        screen.blit(text, (10, 10))
        
        text2 = font.render(
            f"Sensores (Esq/Frente/Dir): {[f'{d:.1f}' for d in robot.sensor_readings]}",
            True, (255, 255, 255)
        )
        screen.blit(text2, (10, 40))
        
        text3 = font.render(
            "W/Seta-Cima: Avançar | S/Seta-Baixo: Recuar | A/Esq: Girar-Esq | D/Dir: Girar-Dir | ESC: Sair",
            True, (200, 200, 200)
        )
        screen.blit(text3, (10, ALTURA - 30))
        
        pygame.display.flip()
    
    pygame.quit()
    print("Simulação encerrada com sucesso!")


if __name__ == "__main__":
    main()
