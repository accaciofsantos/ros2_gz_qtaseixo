import os
import random
import math

# === Configurações básicas ===
dx = 1.5                 # distância base entre árvores
dy = 5.0                 # distância entre fileiras
num_rows = 4
num_cols = 6             # árvores base por fileira
tree_model_uri = "file:///home/criis/ros2_ws/src/ros2_gz_qtaseixo/models/low_poly_tree/model.sdf"

world_folder = "/home/criis/ros2_ws/src/ros2_gz_qtaseixo/worlds"
world_filename = "scenario3_generated.world"
world_path = os.path.join(world_folder, world_filename)

# === Parâmetros ===
yaw_row4 = math.radians(10.0)  # rotação da 4ª fileira
jitter_x = 0.5
jitter_y = 0.5
extra_trees_row3 = 2           # fileira 3 terá 2 árvores a mais

tree_includes = ""
tree_id = 1

for row in range(num_rows):
    # Determine a quantidade de árvores na fileira
    if row == 2:  # 3ª fileira
        cols_in_row = num_cols + extra_trees_row3
    else:
        cols_in_row = num_cols

    y_base = row * dy

    # Pivot para rotação da 4ª fileira (centro da fileira)
    pivot_x = ((cols_in_row - 1) * dx) / 2 if row == 3 else 0
    pivot_y = y_base

    for col in range(cols_in_row):
        x = col * dx

        # 1ª fileira → normal
        if row == 0:
            x2, y2, yaw = x, y_base, 0.0

        # 2ª fileira → jitter
        elif row == 1:
            x2 = x + random.uniform(-jitter_x, jitter_x)
            y2 = y_base + random.uniform(-jitter_y, jitter_y)
            yaw = 0.0

        # 3ª fileira → normal mas com mais árvores (extra árvores já incluídas no col loop)
        elif row == 2:
            x2, y2, yaw = x, y_base, 0.0

        # 4ª fileira → rotacionada 15° em torno do centro
        elif row == 3:
            x_rel = x - pivot_x
            y_rel = y_base - pivot_y
            x2 = pivot_x + (x_rel * math.cos(yaw_row4) - y_rel * math.sin(yaw_row4))
            y2 = pivot_y + (x_rel * math.sin(yaw_row4) + y_rel * math.cos(yaw_row4))
            yaw = 0.0

        # fallback
        else:
            x2, y2, yaw = x, y_base, 0.0

        # Gera o bloco <include>
        tree_includes += f"""    <!-- Tree {tree_id}: row {row+1}, col {col+1} -->
    <include>
      <name>tree_{tree_id}</name>
      <uri>{tree_model_uri}</uri>
      <pose>{x2:.3f} {y2:.3f} 0 0 0 {yaw:.3f}</pose>
    </include>\n"""
        tree_id += 1

# === HEADER do mundo ===
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

# === Combina e salva ===
sdf_content = world_header + tree_includes + world_footer
os.makedirs(world_folder, exist_ok=True)
with open(world_path, "w") as f:
    f.write(sdf_content)

print(f"[OK] scenario3_generated gerado em {world_path}")
