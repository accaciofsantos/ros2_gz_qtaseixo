# Configurações do pomar
dx = 1.5       # distância entre árvores na mesma fileira
dy = 4.0       # distância entre fileiras
num_rows = 4   # número de fileiras
num_cols = 6   # árvores por fileira

# Gerando os waypoints
self.goals = []

# Coordenadas das extremidades (lado de fora)
x_start = -dx/2
x_end = (num_cols-1)*dx + dx/2

for row in range(num_rows):
    y = row*dy
    # Se linha for par, vai do lado direito para o esquerdo
    if row % 2 == 0:
        self.goals.append((x_end, y))
        self.goals.append((x_start, y))
    else:  # linha ímpar, volta do lado esquerdo para o direito
        self.goals.append((x_start, y))
        self.goals.append((x_end, y))

# Retorno ao ponto inicial
self.goals.append((x_end, 0))

# Exemplo de saída
for i, g in enumerate(self.goals):
    print(f"Goal {i}: {g}")
