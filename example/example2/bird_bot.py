# # import mujoco 
# # from mujoco import viewer 
# # import time
# # import numpy as np
# # from numpy import cos, deg2rad

# # XML_PATH = 'example/example2/BirdBot.xml'

# # model = mujoco.MjModel.from_xml_path(XML_PATH)
# # data = mujoco.MjData(model)
# # SIMULATION_TIME = 10

# # A = 100
# # A = deg2rad(100)
# # w = 10
# # tc_0 = 0

# # with mj.viewer.launch_passive(model, data) as viewer:
# #     start = time.time()

# #     while viewer.is_running() and time.time() - start < SIMULATION_TIME:
# #         step_start = time.time()

# #         if step_start - start > 5:
# #             if tc_0 == 0:
# #                 tc_0 = step_start
# #             data.ctrl[1] = A*((1 - cos((step_start - tc_0)*w))/2)

# #         mj.ctrl = A
# #         mj.mj_step(model, data)


# # =======================================================
# # # TEST 2

# # import mujoco 
# # from mujoco import viewer 
# # import time
# # import numpy as np
# # from numpy import cos, deg2rad

# # SIMULATION_TIME = 15            # [sec]
# # XML_FILE = 'example/example2/BirdBot.xml'

# # model = mujoco.MjModel.from_xml_path(XML_FILE)
# # data = mujoco.MjData(model)
# # x = []
# # dx = []
# # t = []

# # A = deg2rad(100)
# # w = 10
# # tc_0 = 0

# # with mujoco.viewer.launch_passive(model, data) as viewer:
# #     start = time.time()
# #     viewer.sync()
# #     while viewer.is_running() and time.time() - start < SIMULATION_TIME:
# #         step_start = time.time()

# #         # if step_start - start > 5:
# #         #     if tc_0 == 0:
# #         #         tc_0 = step_start
# #         #     data.ctrl[1] = A*((1 - cos((step_start - tc_0)*w))/2)
# #         if tc_0 == 0:
# #             tc_0 = step_start
# #         data.ctrl[1] = A*((1 - cos((step_start - tc_0)*w))/2)
# #         # data.ctrl[1] = A
# #         mujoco.mj_step(model, data)


# #         with viewer.lock():
# #             viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

# #         viewer.sync()
# #         current_time = time.time()
# #         # time_until_next_step = model.opt.timestep - (current_time - step_start)
# #         x.append(data.qpos.copy())
# #         dx.append(data.qvel.copy())
# #         t.append(current_time - start)


# #         # if time_until_next_step > 0:
# #         #     time.sleep(time_until_next_step)
# # viewer.close()


# # ===========================================
# # TEST 3

# import time
# import mujoco
# import mujoco.viewer
# import numpy as np

# SIMULATION_TIME = 30

# XML_FILE = 'data/a1/xml/a1.xml'

# model = mujoco.MjModel.from_xml_path(XML_FILE)
# data = mujoco.MjData(model)

# data_file = 'data/a1/walk/stand_twist.csv'
# data_array = np.genfromtxt(data_file, delimiter=',', skip_header=100, skip_footer=100)

# timespan = data_array[:, 0] - data_array[0, 0]
# quaternion = data_array[:, 1:5]
# # linear_velocities = data_array[:, 5:8]
# joint_angles =  data_array[:, 11:23]

# # Set initial position
# data.qpos[0] = 0.
# data.qpos[1] = 0.
# data.qpos[2] = 0.3

# mujoco.mj_resetData(model, data)  
# mujoco.mj_step(model, data)


# # import mujoco as mj
# # import mujoco.viewer
# import time
# from numpy import deg2rad, cos

# # # XML_PATH = 'example/example2/BirdBot.xml'
# # XML_FILE = 'data/a1/xml/a1.xml'

# # SIMULATION_TIME = 15

# # model = mj.MjModel.from_xml_path(XML_FILE)
# # data = mj.MjData(model)


# # timespan = data[:, 0] - data[0, 0]
# # quaternion = data[:, 1:5]
# # joint_angles =  data[:, 11:23]

# # # Set initial position
# # data.qpos[0] = 0.
# # data.qpos[1] = 0.
# # data.qpos[2] = 0.3


# A = 100
# A = deg2rad(100)
# w = 10
# tc_0 = 0

# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()

#     while viewer.is_running() and time.time() - start < SIMULATION_TIME:

#         step_start = time.time()

#         # if step_start - start > 5:
#         #     if tc_0 == 0:
#         #         tc_0 = step_start
#         #     data.ctrl[1] = A*((1 - cos((step_start - tc_0)*w))/2)
#         # if tc_0 == 0:
#         #     tc_0 = step_start
#         # data.ctrl[1] = A*((1 - cos((step_start - tc_0)*w))/2)
#         # # data.ctrl[1] = A
#         # mujoco.mj_step(model, data)
#         # viewer.sync()




#         # step_start = time.time()
    
#         for i in range(len(timespan)):
#             mujoco.mj_step(model, data)
#             data.qpos[0] = 0.
#             data.qpos[1] = 0.
#             data.qpos[2] = 0.3
#             data.qpos[3:7] = quaternion[i]
#             data.qpos[7:] = joint_angles[i]

    
#             data.ctrl = 1
      
           
#             viewer.sync()



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
            # # data.qpos[3:7] = quaternion[i]
            # # data.qpos[7:] = joint_angles[i]
            # data.qpos[3:7] = quaternion[0]
            # data.qpos[7:] = joint_angles[0]

            # dt = time.time() - step_start
            data.ctrl = [100, -100, 100, 0, 0, 100, -100, 100, 0, 0, 10, 10]
            # print(data.ctrl)
       

            # mujoco.mj_kinematics(model, data)
            viewer.sync()