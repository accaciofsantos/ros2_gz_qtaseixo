# Configurações
dx = 1.5           # distância entre árvores na mesma fileira
dy = 4.0           # distância entre fileiras
num_rows = 4       # número de fileiras
num_cols = 6       # árvores por fileira

# Função para gerar o caminho
def generate_ugv_path(dx, dy, num_rows, num_cols):
    goals = []
    for row in range(num_rows):
        y = row * dy + dy/2   # centro entre as fileiras
        if row % 2 == 0:
            # fileira "da esquerda para a direita"
            for col in range(num_cols-1):
                x = col * dx + dx/2  # centro entre árvores
                goals.append((x, y))
        else:
            # fileira "da direita para a esquerda"
            for col in reversed(range(num_cols-1)):
                x = col * dx + dx/2
                goals.append((x, y))
    return goals

# Gerar a lista
path_points = generate_ugv_path(dx, dy, num_rows, num_cols)

# Formatar como self.goals
print("self.goals = [")
for x, y in path_points:
    print(f"    ({x:.2f}, {y:.2f}),")
print("]")
