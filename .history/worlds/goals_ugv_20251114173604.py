# Configurações do pomar
dx = 1.5           # distância entre árvores da mesma fileira
dy = 4.0           # distância entre fileiras
num_rows = 4       # número de fileiras
num_cols = 6       # árvores por fileira

# Ajuste do espaço de segurança lateral do UGV (distância extra para contornar)
margin = 0.5

# Calcula as coordenadas das bordas das fileiras
x_start = 0 - margin
x_end = (num_cols - 1) * dx + margin
y_start = 0 - margin
y_end = (num_rows - 1) * dy + margin

# Inicializa lista de goals
goals = []

# Começa do lado de fora da primeira fileira (direita)
goals.append( (x_end, y_start) )

# Percorre entre as fileiras
for row in range(num_rows - 1):
    # Passagem entre fileira row e row+1
    y_between = row * dy + dy / 2
    # Vai da direita para a esquerda
    goals.append( (x_end, y_between) )
    goals.append( (x_start, y_between) )

# Contorna a última fileira pelo lado de fora
goals.append( (x_start, y_end) )

# Retorna ao ponto inicial
goals.append( (x_end, y_start) )

# Converte para formato do seu script
print("self.goals = [")
for g in goals:
    print(f"    ({g[0]:.2f}, {g[1]:.2f}),")
print("]")
