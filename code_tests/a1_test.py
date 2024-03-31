import mujoco as mj
import mujoco.viewer
import numpy as np
import time

robot_name = "a1"

robot_xml = f"data/a1/xml/a1.xml"

model = mj.MjModel.from_xml_path(robot_xml)
data = mj.MjData(model)
renderer = mj.Renderer(model)


label = f'stand_twist'
data_file = 'data/a1/walk/' + label + '.csv'
# data_file = 'data/a1/walk/stand_twist.csv'

data_array = np.genfromtxt(data_file, delimiter=',', skip_header=100, skip_footer=100)
timespan = data_array[:, 0] - data_array[0, 0]
sampling = np.mean(np.diff(timespan))
quaternion = data_array[:, 1:5]
linear_velocity = data_array[:, 5:8]
joint_angles = data_array[:, 11:23] 
joint_velocity = data_array[:, 23:35] 
joint_torques = data_array[:, 35:47]
contacts = data_array[:, 47:]
final_time = timespan[-1]


print(final_time)

mj.mj_resetData(model, data)
mj.mj_step(model, data)

data.qpos[2] = 0.3
data.qpos[3:7] = quaternion[0]
data.qpos[7:] = joint_angles[0]

mj.mj_step(model, data)

renderer.update_scene(data)


viewer = mj.viewer.launch_passive(
    model,
    data,
    show_left_ui=False,
    show_right_ui=False,
)

# duration = 500  # (seconds)

mj.mj_resetData(model, data)

# while data.time < final_time:
#     # data.ctrl = (np.random.rand(model.nu) - 0.5) * 1

#     # mj.mj_step(model, data)
#     viewer.sync()

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    viewer.sync()
    # while viewer.is_running() and time.time() - start < final_time:
    #     step_start = time.time()
    #     mujoco.mj_step(model, data)
    #     viewer.sync()

viewer.close()






# import mujoco
# from mujoco import viewer
# from numpy import sin, arcsin, deg2rad
# import matplotlib.pyplot as plt
# import time

# SIMULATION_TIME = 5            # [sec]
# XML_FILE = 'BirdBot.xml'

# model = mujoco.MjModel.from_xml_path(XML_FILE)
# data = mujoco.MjData(model)
# x = []
# dx = []
# t = []

# A = deg2rad(70)
# tc_0 = 0
# th0 = 1
# th1 = th0 + 0.3

# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     viewer.sync()
#     while viewer.is_running() and time.time() - start < SIMULATION_TIME:
#         step_start = time.time()

#         if step_start - start > th0 and step_start - start < th1:
#             data.ctrl[1] = A
#         elif step_start - start > th1 and data.ctrl[1] > 0:
#             if tc_0 == 0:
#                 tc_0 = step_start - start
#             data.ctrl[1] = A - 10*(step_start - start - tc_0)
#         else:
#             data.ctrl[1] = 0
            
#         mujoco.mj_step(model, data)

#         with viewer.lock():
#             viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

#         viewer.sync()
#         current_time = time.time()
#         time_until_next_step = model.opt.timestep - (current_time - step_start)
#         x.append(data.qpos.copy())
#         dx.append(data.qvel.copy())
#         t.append(current_time - start)
#         if time_until_next_step > 0:
#             time.sleep(time_until_next_step)
# viewer.close()
# print(step_start - start)
      
# f, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
# ax1.plot(t, x)
# ax1.set_ylabel('Position')
# ax1.grid()
# ax2.plot(t, dx)    
# ax2.set_ylabel('Velocity')
# ax2.grid()
# ax2.set_xlabel('Time')
# plt.show()