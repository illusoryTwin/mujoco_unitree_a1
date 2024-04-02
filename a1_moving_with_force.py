import time
import mujoco
import mujoco.viewer
import numpy as np

SIMULATION_TIME = 50

XML_FILE = 'data/a1/xml/a1.xml'

model = mujoco.MjModel.from_xml_path(XML_FILE)
data = mujoco.MjData(model)

data_file = 'data/a1/walk/stand_twist.csv'
data_array = np.genfromtxt(data_file, delimiter=',', skip_header=100, skip_footer=100)

timespan = data_array[:, 0] - data_array[0, 0]
print("timespan", timespan)
quaternion = data_array[:, 1:5]
joint_angles =  data_array[:, 11:23]


data.qpos[0] = 0.
data.qpos[1] = 0.
data.qpos[2] = 0.3

mujoco.mj_resetData(model, data)  
mujoco.mj_step(model, data)


with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    data.qfrc_applied = np.sin(data.time)

    while viewer.is_running() and time.time() - start < SIMULATION_TIME:
        step_start = time.time()

        for i in range(len(timespan)):

            mujoco.mj_step(model, data)
            # data.qfrc_applied = np.sin(data.time)

            data.qpos[0] = 0.
            data.qpos[1] = 0.
            data.qpos[2] = 0.3
            data.qpos[3:7] = quaternion[i]
            data.qpos[7:] = joint_angles[i]

            # data.qfrc_applied = np.sin(data.time)
            mujoco.mj_step(model, data) 

            mujoco.mj_kinematics(model, data)
            viewer.sync()









# import time
# import mujoco as mj
# import mujoco.viewer
# import numpy as np

# SIMULATION_TIME = 30

# XML_FILE = "data/a1/xml/a1.xml"

# model = mujoco.MjModel.from_xml_path(XML_FILE)
# data = mujoco.MjData(model)
# renderer = mj.Renderer(model)

# viewer = mj.viewer.launch_passive(
#     model,
#     data,
#     show_left_ui=False,
#     show_right_ui=False,
# )

# duration = 500  # (seconds)
# mj.mj_resetData(model, data)
# # while data.time < duration:
# #     data.xfrc_applied[1][:3] = 1e2
# #     mj.mj_step(model, data)

# #     viewer.sync()

# while data.time < duration:
#     if data.time - int(data.time) < 1e-2:
#         data.xfrc_applied[1][:3] = 12
#     mj.mj_step(model, data)

#     viewer.sync()
