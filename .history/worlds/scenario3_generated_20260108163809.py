import os
import random
import math

# === Configurações ===
dx = 1.5                 # distância entre árvores na mesma fileira
dy = 4.0                 # distância entre fileiras
num_rows = 4
num_cols = 6
tree_model_uri = "file:///home/criis/ros2_ws/src/ros2_gz_qtaseixo/models/low_poly_tree/model.sdf"

world_folder = "/home/criis/ros2_ws/src/ros2_gz_qtaseixo/worlds"
world_filename = "scenario3_generated.world"
world_path = os.path.join(world_folder, world_filename)

# === Parâmetros do desalinhamento ===
yaw_col4 = math.radians(5.0)   # 5° de inclinação no plano XY para a 4ª coluna
jitter_x = 0.20                # deslocamento aleatório ±0.20m para colunas 2 e 3
jitter_y = 0.25                # deslocamento aleatório ±0.25m para colunas 2 e 3

tree_includes = ""
tree_id = 1

for row in range(num_rows):
    y = row * dy
    for col in range(num_cols):
        x = col * dx

        # === 1ª coluna: base, sem alterações ===
        if col == 0:
            x2, y2, yaw = x, y, 0.0

        # === 4ª coluna: desalinhada, aplica rotação no plano XY ===
        elif col == 3:
            # Rotaciona a árvore em torno da origem (0,0) no plano XY
            x2 = x * math.cos(yaw_col4) - y * math.sin(yaw_col4)
            y2 = x * math.sin(yaw_col4) + y * math.cos(yaw_col4)
            yaw = yaw_col4

        # === 2ª e 3ª colunas: pequenas variações aleatórias (jitter) ===
        elif col in [1, 2]:
            x2 = x + random.uniform(-jitter_x, jitter_x)
            y2 = y + random.uniform(-jitter_y, jitter_y)
            yaw = 0.0

        # fallback
        else:
            x2, y2, yaw = x, y, 0.0

        # === Gera o bloco <include> da árvore ===
        tree_includes += f"""    <!-- Tree {tree_id}: row {row+1}, col {col+1} -->
    <include>
      <name>tree_{tree_id}</name>
      <uri>{tree_model_uri}</uri>
      <pose>{x2:.3f} {y2:.3f} 0 0 0 {yaw:.3f}</pose>
    </include>\n"""
        tree_id += 1

# === HEADER do mundo (igual ao scenario2) ===
world_header = """<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='scenario3'>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>41.1780</latitude_deg>
      <longitude_deg>-8.5980</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

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

    <physics type='ode'>
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

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
"""

world_footer = "  </world>\n</sdf>"

sdf_content = world_header + tree_includes + world_footer

# === Salva o arquivo SDF ===
os.makedirs(world_folder, exist_ok=True)
with open(world_path, "w") as f:
    f.write(sdf_content)

print(f"[OK] scenario3_generated gerado em {world_path}")
