# AULA 04 - Laboratorios (AC-2 Parte 1)

## Identificacao

- Disciplina: Robotica / Cinematica e Controle
- Integrantes: preencher com os nomes da dupla/trio
- Data: 14/09/2026

## Como executar

Na pasta `AULA_04`, instale as dependencias com `pip install pygame numpy` e execute:

```bash
python3 lab1_pose.py
python3 lab2_ackermann.py
python3 lab3_sensor_filter.py
python3 lab4_braitenberg.py
python3 ex5_corridor.py
```

Para verificar os calculos sem abrir a janela grafica, use `--headless`.

## Exercicio 1 - Validador de pose

O modelo diferencial usa $x' = v cos(theta)$, $y' = v sin(theta)$ e $theta' = omega$. O primeiro trecho percorre 2 m no eixo X, o segundo gira aproximadamente 90 graus, pois `0.7854 * 2 = 1.5708 rad`, e o terceiro percorre 1.2 m na nova orientacao. Assim, a pose teorica esperada e aproximadamente `(x=2.0000, y=1.2000, theta=1.5708 rad)`.

**Print da execucao:**

```text
Pose final teorica: x=2.0000 m, y=1.2000 m, theta=1.5708 rad (90.00 deg)
Pose simulada:      x=2.0000 m, y=1.2000 m, theta=1.5708 rad (90.00 deg)
```

> Inserir tambem o print da janela Pygame durante a execucao.

**Conclusao:** a pose simulada usa passos de 10 ms e deve coincidir com a integracao teorica, validando a aplicacao dos comandos temporizados em malha aberta.

## Exercicio 2 - Ackermann versus diferencial

Para Ackermann, `omega = (v/L) * tan(phi)` e `R = L/tan(phi)`. Com `L=2.0 m` e `phi` limitado a +/-30 graus, o menor raio possivel em modulo e `2/tan(30 graus) = 3.464 m`. Portanto, o raio nao pode ser zero. No robo diferencial, por outro lado, e possivel girar no proprio eixo com velocidade linear nula e velocidades das rodas de sinais opostos.

**Print da execucao:**

```text
Ackermann: v=1.00 m/s, phi=20.00 deg, omega=0.1820 rad/s, R=5.4950 m
Limite aplicado: phi entre -30.00 e +30.00 graus
```

> Inserir tambem o print da trajetoria circular no Pygame.

**Conclusao:** aumentar o modulo de `phi` aumenta a curvatura e reduz o raio; em `phi=0`, o raio e infinito e a trajetoria e reta.

## Exercicio 3 - Filtro de alcance

A varredura usa sete angulos igualmente espacados de `-pi/2` a `+pi/2`. O ruido e gaussiano com media zero e desvio padrao 5 px. O threshold descarta leituras abaixo de 10 px e satura as maiores que 200 px usando `clip`, mantendo o valor tratado no intervalo `[10, 200]`.

**Print da execucao:**

```text
Feixe 1 ( -90.0 deg): bruto=31.74 px | tratado=31.74 px
Feixe 2 ( -60.0 deg): bruto=74.13 px | tratado=74.13 px
Feixe 3 ( -30.0 deg): bruto=148.32 px | tratado=148.32 px
Feixe 4 (   0.0 deg): bruto=183.30 px | tratado=183.30 px
Feixe 5 (  30.0 deg): bruto=131.79 px | tratado=131.79 px
Feixe 6 (  60.0 deg): bruto=74.97 px | tratado=74.97 px
Feixe 7 (  90.0 deg): bruto=31.88 px | tratado=31.88 px
```

> Inserir tambem o print da tela Pygame.

**Conclusao:** o controlador deve receber o valor tratado, enquanto o valor bruto permanece visivel para comparar o efeito do filtro.

## Exercicio 4 - Braitenberg direto

As conexoes sao diretas: sensor esquerdo para roda esquerda e sensor direito para roda direita. A lei implementada e `vL = v0 + alpha*(1-d_esq/d_max)` e `vR = v0 + alpha*(1-d_dir/d_max)`. Se o obstaculo esta a direita, `d_dir` diminui, `vR` aumenta, e o robo curva para a direita. Esse e o comportamento de atracao/agressao, oposto ao de aversao.

**Print da execucao:**

```text
Obstaculo a direita: vL=44.00 px/s, vR=72.12 px/s
Como vR > vL, o robo curva para a direita: comportamento de atracao/agressao.
```

> Inserir tambem o print da arena com o obstaculo.

## Exercicio 5 - Centralizacao no corredor

As leituras laterais sao `d_esq = y - parede_esquerda` e `d_dir = parede_direita - y`. O erro `e = d_esq - d_dir` alimenta o controlador proporcional `omega = 0.01*e`, enquanto a velocidade linear fica constante em 40 px/s. O sinal do erro altera a orientacao para reduzir a diferenca entre as distancias.

**Print da execucao:**

```text
Controlador: e = d_esq - d_dir; omega = Kp * e; Kp=0.010; v=40.0 px/s
Estado inicial: y=170.00 px | estado final: y=298.65 px | erro final=-2.74 px
```

> Inserir tambem o print da simulacao com a posicao inicial desalinhada e o percurso corrigido.

**Conclusao:** por ser malha fechada, o controlador mede novamente as distancias a cada ciclo e corrige o desvio continuamente, mantendo o robo proximo ao centro do corredor.
