# Configurações
dx = 1.5           # distância entre árvores na mesma fileira
dy = 4.0           # distância entre fileiras
num_rows = 4       # número de fileiras
num_cols = 6       # árvores por fileira

# Gerar caminho contornando externamente e depois serpenteando
def generate_ugv_path_perimeter(dx, dy, num_rows, num_cols):
    goals = []

    # Coordenadas das fileiras e colunas
    x_coords = [i * dx + dx/2 for i in range(num_cols)]
    y_coords = [i * dy + dy/2 for i in range(num_rows)]

    # Circundar o perímetro externo (primeira volta)
    # canto inferior esquerdo -> inferior direito
    for x in x_coords:
        goals.append((x, y_coords[0]))
    # inferior direito -> canto superior direito
    for y in y_coords[1:]:
        goals.append((x_coords[-1], y))
    # canto superior direito -> canto superior esquerdo
    for x in reversed(x_coords[:-1]):
        goals.append((x, y_coords[-1]))
    # canto superior esquerdo -> canto inferior esquerdo
    for y in reversed(y_coords[1:-1]):
        goals.append((x_coords[0], y))

    # Depois, serpenteando internamente pelas fileiras restantes
    for row in range(1, num_rows-1):
        y = y_coords[row]
        if row % 2 == 1:
            for x in reversed(x_coords[1:-1]):
                goals.append((x, y))
        else:
            for x in x_coords[1:-1]:
                goals.append((x, y))

    return goals

# Gerar a lista
path_points = generate_ugv_path_perimeter(dx, dy, num_rows, num_cols)

# Formatar como self.goals
print("self.goals = [")
for x, y in path_points:
    print(f"    ({x:.2f}, {y:.2f}),")
print("]")
