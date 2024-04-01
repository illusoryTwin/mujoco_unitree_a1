# import mujoco as mj
# import mujoco.viewer
# import time

# XML_FILE = 'example/xml/projectile.xml'
# SIMULATION_TIME = 30

# def controller(model, data):
#     pass


# model = mj.MjModel.from_xml_path(XML_FILE)
# data = mj.MjData(model)

# mj.mj_resetData(model, data)
# mj.mj_step(model, data)

# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()

#     while viewer.is_running() and time.time() - start < SIMULATION_TIME:
#         step_start = time.time()
#         mujoco.mj_step(model, data)

#     viewer.sync()


# =========================

import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

xml_path = 'data/a1/xml/a1.xml'

data_file = 'data/a1/walk/stand_twist.csv'
data_array = np.genfromtxt(data_file, delimiter=',', skip_header=100, skip_footer=100)

timespan = data_array[:, 0] - data_array[0, 0]
quaternion = data_array[:, 1:5]
joint_angles =  data_array[:, 11:23]

# 'example/xml/projectile.xml'

#xml file (assumes this is in the same folder as this file)
simend = 10 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)


def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    # pass
    # Force = -c*vx*|v| i + -c*vy*|v| j + -c*vz*|v| k
    vx = data.qvel[0];
    vy = data.qvel[1];
    vz = data.qvel[2];
    v = np.sqrt(vx**2+vy**2+vz**2)
    c = 0.5
    # data.qfrc_applied[0] = -c*vx*v;
    # data.qfrc_applied[1] = -c*vy*v;
    # data.qfrc_applied[2] = -c*vz*v;
    data.xfrc_applied[1][0] = -c*vx*v;
    data.xfrc_applied[1][1] = -c*vy*v;
    data.xfrc_applied[1][2] = -c*vz*v;


model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)    
init_controller(model,data)
data.qpos[0] = 0
data.qpos[2] = 0.1
# data.qvel[0] = 2
# data.qvel[2] = 5


# #get the full path
# dirname = os.path.dirname(__file__)
# abspath = os.path.join(dirname + "/" + xml_path)
# xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# # install GLFW mouse and keyboard callbacks
# glfw.set_key_callback(window, keyboard)
# glfw.set_cursor_pos_callback(window, mouse_move)
# glfw.set_mouse_button_callback(window, mouse_button)
# glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])
cam.azimuth = 90.11676025390628 ; cam.elevation = -50.03149414062499 ; cam.distance =  10
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

#initialize the controller


#set the controller
# mj.set_mjcb_control(controller)

while not glfw.window_should_close(window):
    time_prev = data.time
    # for i in range(len(timespan)):
    i = 0
    while (data.time - time_prev < 1.0/60.0):
        # for i in range(len(timespan)):

        mj.mj_step(model, data)

        data.qpos[0] = 0.
        data.qpos[1] = 0.
        data.qpos[2] = 0.3
        data.qpos[3:7] = quaternion[i]
        data.qpos[7:] = joint_angles[i]

        mj.mj_kinematics(model, data)
        i+=1


    if (data.time>=simend):
        break;

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()




# import mujoco as mj
# from mujoco.glfw import glfw
# import numpy as np
# import os
# import time
# import mujoco.viewer
# import numpy as np

# SIMULATION_TIME = 30

# xml_path = 'example/xml/projectile.xml'

# #xml file (assumes this is in the same folder as this file)
# simend = 10 #simulation time
# print_camera_config = 0 #set to 1 to print camera config
#                         #this is useful for initializing view of the model)


# model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
# data = mj.MjData(model)   

# def init_controller(model,data):
#     #initialize the controller here. This function is called once, in the beginning
#     pass

# def controller(model, data):
#     #put the controller here. This function is called inside the simulation.
#     # pass
#     # Force = -c*vx*|v| i + -c*vy*|v| j + -c*vz*|v| k
#     vx = data.qvel[0];
#     vy = data.qvel[1];
#     vz = data.qvel[2];
#     v = np.sqrt(vx**2+vy**2+vz**2)
#     c = 0.5
#     # data.qfrc_applied[0] = -c*vx*v;
#     # data.qfrc_applied[1] = -c*vy*v;
#     # data.qfrc_applied[2] = -c*vz*v;
#     data.xfrc_applied[1][0] = -c*vx*v;
#     data.xfrc_applied[1][1] = -c*vy*v;
#     data.xfrc_applied[1][2] = -c*vz*v;


# model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
# data = mj.MjData(model)    
# init_controller(model,data)
# data.qpos[0] = 0
# data.qpos[1] = 0
# data.qpos[2] = 0.1
# data.qvel[0] = 2
# data.qvel[2] = 5



# #set the controller
# mj.set_mjcb_control(controller)

# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     data.qpos[0] = 0.
#     data.qpos[1] = 0.
#     data.qpos[2] = 0.3
#     while viewer.is_running() and time.time() - start < SIMULATION_TIME:
#         step_start = time.time()

#         for i in range(1000):
#             mujoco.mj_step(model, data)
#             # mj.set_mjcb_control(controller)

#             # data.qpos[0] = 0.
#             # data.qpos[1] = 0.
#             # data.qpos[2] = 0.3
#             mj.set_mjcb_control(controller)

#             # data.qpos[3:7] = quaternion[i]
#             # data.qpos[7:] = joint_angles[i]

#             # mujoco.mj_kinematics(model, data)
#             viewer.sync()