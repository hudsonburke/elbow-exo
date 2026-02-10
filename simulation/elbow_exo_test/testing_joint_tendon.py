import mujoco 
import mujoco.viewer
import os
import time
import numpy as np

# Load the model
model_path = r"D:\1. Nazirah\MuJoCo\MyoSim_things\elbow_exo_test\elbow\myoelbow_1dof6muscles_1dofexo_tendon_nobars.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Exo")
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "r_elbow_flex")

tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, "McKibben_tendon")
McKibben_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "McKibben")

# print("Current tendon length: ", data.ten_length[tendon_id])

# Optional: view results with viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -30
    viewer.cam.distance = 2.5
    viewer.cam.lookat[:] = [0, 0, 1]
    time.sleep(2)
    print("Starting simulation...")
    
    # start_time = time.time()

    while viewer.is_running():
        # elapsed = time.time() - start_time
        # print("Current tendon length: ", data.ten_length[tendon_id])
        mujoco.mj_step(model, data)
        print("Current McKibben force is ", data.actuator_force[McKibben_id]) # Newtons
        viewer.sync()
