# Python script to generate Gazebo world XML for a polynomial path

# Polynomial function definition
def cubic_polynomial(x):
    return x**3 - 6*x**2 + 9*x -2

# Generate path segments
def generate_path_segments(x_start, x_end, step):
    segments = []
    for x in range(x_start, int(x_end/step)):
        x_pos = x * step
        y_pos = cubic_polynomial(x_pos)
        segment = f'''
    <model name="path_segment_{x}">
      <static>true</static>
      <pose>{x_pos} {y_pos} 0.01 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{step} 0.05 0.01</size>
            </box>
            <material>
              <ambient>0 0 1 1</ambient>
            </material>
          </geometry>
        </visual>
      </link>
    </model>
        '''
        segments.append(segment)
    return segments

# Settings
x_start = 0
x_end = 5
step = 0.01  # Determines the resolution of the path

# Generate the world file content
segments_xml = generate_path_segments(x_start, x_end, step)
world_content = f'''<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="basic_world">
    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- Path segments -->
    {''.join(segments_xml)}
  </world>
</sdf>
'''

# Output to a file (or print to console)
with open("polynomial_path_world.world", "w") as world_file:
    world_file.write(world_content)
