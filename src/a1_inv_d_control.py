
import time
import mujoco
import mujoco.viewer
import numpy as np

SIMULATION_TIME = 30

XML_FILE = 'data/a1/xml/a1.xml'

model = mujoco.MjModel.from_xml_path(XML_FILE)
data = mujoco.MjData(model)

data_file = 'data/a1/walk/stand_twist.csv'
data_array = np.genfromtxt(data_file, delimiter=',', skip_header=100, skip_footer=100)

timespan = data_array[:, 0] - data_array[0, 0]
quaternion = data_array[:, 1:5]
# linear_velocities = data_array[:, 5:8]
joint_angles =  data_array[:, 11:23]

# Set initial position
data.qpos[0] = 0.
data.qpos[1] = 0.
data.qpos[2] = 0.3
data.qpos[3:7] = quaternion[0]
data.qpos[7:] = joint_angles[0]

mujoco.mj_resetData(model, data)  
mujoco.mj_step(model, data)

print("Initial base position:", data.qpos[0:3])

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()

    while viewer.is_running() and time.time() - start < SIMULATION_TIME:
        step_start = time.time()
    
        for i in range(len(timespan)):
            mujoco.mj_step(model, data)
            data.qpos[0] = 0.
            data.qpos[1] = 0.
            data.qpos[2] = 0.3
            data.qpos[3:7] = quaternion[i]
            data.qpos[7:] = joint_angles[0]
            

            data.qacc = 0
            mujoco.mj_inverse(model, data)
            data.ctrl = data.qfrc_inverse[:12]
            
            viewer.sync()