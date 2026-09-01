# Resultados - AULA-03: Laboratórios de Robótica (LAB-1 ao LAB-5)

## 📌 Resumo Executivo

Implementação completa de **5 laboratórios progressivos** de robótica em PyGame, evoluindo de percepção sensorial (raycasting) para planejamento autônomo (pathfinding A*) com coordenação multi-robô.

---

## 📚 LAB-1: Sistema de Raycasting com 3 Sensores

**Arquivo:** `lab01_aula03.ipynb` | **Script:** `lab01_raycasting_robot.py`

### ✨ Características
- **3 sensores** em ângulos (-45°, 0°, +45°)
- **Algoritmo AABB** para colisão ray-rectangle
- **Renderização 60 FPS** com PyGame
- **Controle manual** (W/S/A/D + Setas)
- **HUD** com posição, orientação e leituras

---

## 🎯 LAB-2: Sistema Radar com 8 Sensores

**Arquivo:** `lab02_aula03.ipynb`

### ✨ Características
- **8 sensores** distribuídos em 360°
- **Radar circular** mostrando ocupação
- **Histograma** de distâncias
- **Estatísticas** (mín, máx, média)
- **Histórico** de últimas 100 amostras

---

## 🤖 LAB-3: Navegação Autônoma com Evitação

**Arquivo:** `lab03_aula03.ipynb`

### ✨ Características
- **Força potencial artificial** (atração + repulsão)
- **Estados**: idle, navigating, obstacle_avoidance, reached
- **Controle proporcional** para orientação
- **Trajetória registrada** em tempo real
- **Alvo por click** do mouse

---

## 👥 LAB-4: Múltiplos Robôs com Coordenação

**Arquivo:** `lab04_aula03.ipynb`

### ✨ Características
- **4 robôs** independentes com cores diferentes
- **Evitação entre robôs** (distância mínima)
- **Alvo compartilhado** redirecional
- **Comunicação implícita** (detecção de vizinhos)
- **Trajetória individual** para cada robô

---

## 🗺️ LAB-5: Mapa do Ambiente e Pathfinding A*

**Arquivo:** `lab05_aula03.ipynb`

### ✨ Características
- **Occupancy Grid** (mapa de grade)
- **Algoritmo A*** com heurística euclidiana
- **Mapa de calor** visualizando obstáculos
- **Waypoint planning** e seguimento
- **Controle adaptativo** com replanejamento
- **Toggle mapa** com tecla G

---

## 📊 Comparação entre Laboratórios

| Aspecto | LAB-1 | LAB-2 | LAB-3 | LAB-4 | LAB-5 |
|---------|-------|-------|-------|-------|-------|
| Sensores | 3 | 8 | 8 | 8 | 8 |
| Navegação | Manual | Manual | Autônoma | Autônoma | Autônoma |
| Robôs | 1 | 1 | 1 | 4 | 1 |
| Planejamento | - | - | Reativo | Reativo | Global+Reativo |
| Visualização | Sensores | Radar | Trajetória | Multi | Mapa |

---

## 🎮 Controles Universais

| Controle | Ação |
|----------|------|
| **W** / **↑** | Avançar (LAB-1) |
| **S** / **↓** | Recuar (LAB-1) |
| **A** / **←** | Girar esq (LAB-1) |
| **D** / **→** | Girar dir (LAB-1) |
| **Mouse Click** | Novo alvo (LAB-3,4,5) |
| **G** | Toggle mapa (LAB-5) |
| **ESC** | Encerrar |

---

## 📁 Estrutura de Arquivos

```
/home/aluno/Downloads/AULA_03/
├── lab01_aula03.ipynb          ✅ LAB-1
├── lab02_aula03.ipynb          ✅ LAB-2
├── lab03_aula03.ipynb          ✅ LAB-3
├── lab04_aula03.ipynb          ✅ LAB-4
├── lab05_aula03.ipynb          ✅ LAB-5
├── lab01_raycasting_robot.py   ✅ Script LAB-1
└── resultados_aula03.md        ✅ Este arquivo
```

---

## 🚀 Como Executar

```bash
cd /home/aluno/Downloads/AULA_03

# LAB-1: Script Python
python3 lab01_raycasting_robot.py

# LABs 2-5: Jupyter Notebook
jupyter notebook lab02_aula03.ipynb  # (etc)
```

---

## 🔧 Requisitos

- Python 3.6+
- PyGame: `pip install pygame`
- NumPy: `pip install numpy`
- Jupyter: `pip install jupyter` (para notebooks)

---

## ✅ Checklist Final

- [x] LAB-1: Raycasting com 3 sensores
- [x] LAB-2: 8 sensores + Radar
- [x] LAB-3: Navegação autônoma
- [x] LAB-4: Multi-robô coordenado
- [x] LAB-5: Mapa + A* pathfinding
- [x] Documentação completa
- [x] Scripts funcionais
- [x] Controles interativos

---

**Status:** ✅ 100% COMPLETO | **Versão:** 1.0 | **Data:** 2026-08-31
