# import mujoco as mj
# from mujoco import viewer 

# XML_FILE = 'data/a1/xml/a1.xml'
# model = mj.MjModel.from_xml_path(XML_FILE)
# data = mj.MjData(model)

# with mj.viewer.launch_passive(model, data) as viewer:
#     viewer.sync()



# ================================================================
# import time

# import mujoco
# import mujoco.viewer
# from robot_descriptions import go1_mj_description

# SIMULATION_TIME = 30
# # XML_FILE = 'data/a1/xml/a1.xml'

# # m = mujoco.MjModel.from_xml_path(XML_FILE)

# model = mujoco.MjModel.from_xml_path(go1_mj_description.MJCF_PATH)
# data = mujoco.MjData(model)

# with mujoco.viewer.launch_passive(model, data) as viewer:
#   # Close the viewer automatically after 30 wall-seconds.
#   start = time.time()
#   while viewer.is_running() and time.time() - start < SIMULATION_TIME:
#     step_start = time.time()

#     mujoco.mj_step(model, data)

#     viewer.sync()
# ===================================================================




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
# data.qpos[2] = 0.3
# data.qpos[3:7] = quaternion[0]
# data.qpos[7:] = joint_angles[0]
# mujoco.mj_step(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < SIMULATION_TIME:
    step_start = time.time()

    # ==================
    # mujoco.mj_resetData(model, data)  # Reset state and time.
    # mujoco.mj_step(model, data)

    # data.qpos[2] = 0.3
    # data.qpos[3:7] = quaternion[0]
    # data.qpos[7:] = joint_angles[0]
    # mujoco.mj_step(model, data)
    # ===================
    mujoco.mj_resetData(model, data)  # Reset state and time.
    mujoco.mj_step(model, data)
    model.opt.timestep = 1/1000

    framerate = 30  # (Hz)
    data.qpos[0] = 0. # x
    data.qpos[1] = 0. # y
    data.qpos[2] = 0.3 # z 
    # mujoco.mj_step(model, data)

    # Example modification of a viewer option: toggle contact points every two seconds.
    # with viewer.lock():
    #   viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    # time_until_next_step = m.opt.timestep - (time.time() - step_start)
    # if time_until_next_step > 0:
    #   time.sleep(time_until_next_step)