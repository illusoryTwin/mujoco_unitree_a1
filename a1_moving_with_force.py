import mujoco 
import mujoco.viewer
import time
from mujoco.glfw import glfw
import numpy as np
import os

XML_FILE = 'data/a1/xml/a1.xml'

SIMULATION_TIME = 10 

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):

    x = data.qpos[0];
    y = data.qpos[1];
    z = data.qpos[2];

    v = np.sqrt(x**2+y**2+z**2)
    c = 0.5
    # data.qfrc_applied[0] = -c*vx*v;
    # data.qfrc_applied[1] = -c*vy*v;
    # data.qfrc_applied[2] = -c*vz*v;
    data.xfrc_applied[1][0] = -c*x*v;
    data.xfrc_applied[1][1] = -c*y*v;
    data.xfrc_applied[1][2] = -c*z*v;

# MuJoCo data structures
model = mujoco.MjModel.from_xml_path(XML_FILE)  
data = mujoco.MjData(model)                


data_file = 'data/a1/walk/stand_twist.csv'

data_array = np.genfromtxt(data_file, delimiter=',', skip_header=100, skip_footer=100)

timespan = data_array[:, 0] - data_array[0, 0]
quaternion = data_array[:, 1:5]
joint_angles =  data_array[:, 11:23]


#initialize the controller
init_controller(model,data)
data.qpos[0] = 0
data.qpos[2] = 0.1
data.qvel[0] = 2
data.qvel[2] = 5

#set the controller
mujoco.set_mjcb_control(controller)

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()

    while viewer.is_running() and time.time() - start < SIMULATION_TIME:
        step_start = time.time()

        for i in range(len(timespan)):
            mujoco.mj_step(model, data)
            viewer.sync()
