import time

import mujoco
import mujoco.viewer
from robot_descriptions import go1_mj_description
import numpy as np

SIMULATION_TIME = 30

XML_FILE = 'data/a1/xml/a1.xml'

model = mujoco.MjModel.from_xml_path(XML_FILE)

# model = mujoco.MjModel.from_xml_path(go1_mj_description.MJCF_PATH)
renderer = mujoco.Renderer(model)
data = mujoco.MjData(model)

data_file = 'data/a1/walk/stand_twist.csv'
data_array = np.genfromtxt(data_file, delimiter=',', skip_header=100, skip_footer=100)

timespan = data_array[:, 0] - data_array[0, 0]
sampling = np.mean(np.diff(timespan))
quaternion = data_array[:, 1:5]
linear_velocity = data_array[:, 5:8]
joint_angles =  data_array[:, 11:23]
joint_velocity = data_array[:, 23:35]
joint_torques = data_array[:, 35:47]
contacts = data_array[:, 47:]
final_time = timespan[-1]

mujoco.mj_resetData(model, data)  # Reset state and time.
mujoco.mj_step(model, data)

print("data.qpos.shape", data.qpos.shape, "data.qvel.shape", data.qvel.shape)

# data.qpos[0] = 0.
# data.qpos[1] = 0.
# data.qpos[2] = 0.3
# data.qpos[3:7] = quaternion[0]
# data.qpos[7:] = joint_angles[0]

data.qpos[2] = 0.3
data.qpos[3:7] = quaternion[0]
data.qpos[7:] = joint_angles[0]


mujoco.mj_step(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < SIMULATION_TIME:
    step_start = time.time()

    # mujoco.mj_resetData(model, data)  # Reset state and time.
    # mujoco.mj_step(model, data)

    # for i in range(1000):
    #     mujoco.mj_resetData(model, data)  # Reset state and time.
    #     mujoco.mj_step(model, data)
    #     time_ = timespan[i]

        # data.qpos[3:7] = quaternion[i]
        # data.qpos[7:] = joint_angles[i]
        # data.qvel[0:3] = linear_velocity[i]
        # data.qvel[3:6] = linear_velocity[i]
        # data.qvel[6:9] = linear_velocity[i]

        # mujoco.mj_kinematics(model, data)
        # mujoco.mj_step(model, data)

    viewer.sync()# num_columns = data_array.shape[1]

