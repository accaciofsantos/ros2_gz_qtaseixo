import os

# Configurações de espaçamento e quantidade
dx = 1.5           # distância entre árvores na mesma fileira
dy = 4.0           # distância entre fileiras
num_rows = 4       # número de fileiras
num_cols = 6       # árvores por fileira
tree_model_uri = "file:///home/criis/ros2_ws/src/ros2_gz_qtaseixo/models/low_poly_tree/model.sdf"

# Caminho da pasta do world
world_folder = "/home/criis/ros2_ws/src/ros2_gz_qtaseixo/worlds"
world_filename = "scenario2_generated.world"
world_path = os.path.join(world_folder, world_filename)

# Gerar blocos <include> das árvores
tree_includes = ""
tree_id = 1
for row in range(num_rows):
    y = row * dy
    for col in range(num_cols):
        x = col * dx
        tree_includes += f"""    <!-- Árvore {tree_id}: linha {row+1}, coluna {col+1} -->
    <include>
      <name>tree_{tree_id}</name>
      <uri>{tree_model_uri}</uri>
      <pose>{x} {y} 0 0 0 0</pose>
    </include>\n"""
        tree_id += 1

# Cabeçalho do mundo (plugins, física e chão)
world_header = """<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='scenario2'>

    <!-- Coordenadas geográficas para NavSat -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>41.1780</latitude_deg>
      <longitude_deg>-8.5980</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <!-- Plugins principais -->
    <plugin name='gz::sim::systems::Physics' filename='libgz-sim-physics-system.so'/>
    <plugin name='gz::sim::systems::UserCommands' filename='libgz-sim-user-commands-system.so'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='libgz-sim-scene-broadcaster-system.so'/>
    <plugin name="gz::sim::systems::Sensors" filename="libgz-sim-sensors-system.so">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::Imu" filename="libgz-sim-imu-system.so"/>
    <plugin name="gz::sim::systems::NavSat" filename="libgz-sim-navsat-system.so"/>
    <plugin name="gz::sim::systems::Magnetometer" filename="libgz-sim-magnetometer-system.so"/>
    <plugin name="gz::sim::systems::AirPressure" filename="libgz-sim-air-pressure-system.so"/>
    <plugin name="gz::sim::systems::Contact" filename="libgz-sim-contact-system.so"/>

    <!-- Física -->
    <physics type='ode'>
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Chão -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>70 70</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>70 70</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.7 0.3 1</ambient>
            <diffuse>0.3 0.7 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- === Fileiras de árvores geradas automaticamente === -->
    <!-- dx = {dx} m entre árvores da mesma fileira -->
    <!-- dy = {dy} m entre fileiras -->
    <!-- Número de fileiras = {num_rows}, árvores por fileira = {num_cols} -->
"""

world_footer = "  </world>\n</sdf>"

# Combinar tudo
sdf_content = world_header + tree_includes + world_footer

# Criar pasta caso não exista
os.makedirs(world_folder, exist_ok=True)

# Salvar o arquivo diretamente na pasta do world
with open(world_path, "w") as f:
    f.write(sdf_content)

print(f"Arquivo SDF gerado com sucesso em: {world_path}")
