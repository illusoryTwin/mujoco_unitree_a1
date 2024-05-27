
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

print("len(data.qpos)", len(data.qpos))
print("len(data.qvel)", len(data.qvel))
print("len(data.qacc)", len(data.qacc))

print(data. qpos)
print("data.ctrl", data.ctrl)

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

    step_start = time.time()
    data.qpos[0] = 0.
    data.qpos[1] = 0.
    data.qpos[2] = 0.3
    data.qpos[3:7] = quaternion[0]
    data.qpos[7:] = joint_angles[0]
    mujoco.mj_step(model, data)
    print("data.ctrl", data.ctrl)
    data.qacc = 0
    mujoco.mj_inverse(model, data)
    qfrc0 = data.qfrc_inverse

    ctrl0 = np.atleast_2d(qfrc0) @ np.linalg.pinv(data.actuator_moment)
    ctrl0 = ctrl0.flatten()  # Save the ctrl setpoint.

    # while viewer.is_running() and time.time() - start < SIMULATION_TIME:

    #     for i in range(len(timespan)):    

    #         # print("len(data.qpos)", len(data.qpos))

    #         # print("len(data.qacc)", len(data.qacc))
    #         # print("len(data.qvel)", len(data.qvel))
    #         # print(data.qfrc_applied)
    #         # # print(len(data.xfrc_applied))
    #         # print(data.qfrc_actuator)

            
    #         print('control setpoint:', ctrl0)
    #         print("data.qfrc_inverse", data.qfrc_inverse)
    #         data.ctrl = ctrl0
    #         # data.ctrl = np.random.randn(12)
    #         # data.ctrl = [0]*12

    #         mujoco.mj_step(model, data)

    #         viewer.sync()
                        