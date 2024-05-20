
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
            # data.qpos[3:7] = quaternion[i]
            data.qpos[7:] = joint_angles[0]


            # data.qacc = [0]*18
            # mujoco.mj_inverse(model, data)
            # data.ctrl = data.qfrc_inverse[:12]



            # # another method
            mujoco.mj_forward(model, data)
            data.qacc = 0
            mujoco.mj_inverse(model, data)
            print("data.qfrc_inverse", data.qfrc_inverse)

            # # define control and configuration to linearize around
            # print(data.qfrc_actuator)





            print("len(data.qM.shape)", len(data.qM)) # problem here

            # print("len(data.qfrc_bias)", len(data.qfrc_bias))

            # print("data.qfrc_applied", data.qfrc_applied)
            # print("data.qfrc_inverse", data.qfrc_inverse)

            # print("len(data.qpos)", len(data.qvel))
            # print("len(data.qfrc_inverse[:12])", len(data.qfrc_inverse[:12]))
            # print("len(data.ctrl)", len(data.ctrl))
            # print(data.qfrc_inverse)

            nu = model.nu
            nv = model.nv

        # Allocate the A and B matrices, compute them.
            A = np.zeros((2*nv, 2*nv))
            B = np.zeros((2*nv, nu))
            epsilon = 1e-6
            centered = True
            mujoco.mjd_transitionFD(model, data, epsilon, centered, A, B, None, None)
            # print("A", A.shape) # (36, 36)
            # print("B", B.shape) # (36, 12)
            

            viewer.sync()