import mujoco 
import mujoco.viewer
import os
import time
import numpy as np

# Load the model
model_path = r"D:\1. Nazirah\MuJoCo\MyoSim_things\elbow_exo_test\elbow\myoelbow_1dof6muscles_1dofexo.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "Exo")
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "r_elbow_flex")

desired_torque = 10.0  # Newton-meters
desired_time = 5.0     # Seconds
gear_ratio = model.actuator_gear[act_id][0]
data.ctrl[act_id] = desired_torque / gear_ratio



# Optional: view results with viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -30
    viewer.cam.distance = 2.5
    viewer.cam.lookat[:] = [0, 0, 1]
    time.sleep(5)
    print("Applying ", desired_torque, " Nm torque for ", desired_time, " seconds...")

    start_time = time.time()

    while viewer.is_running():
        elapsed = time.time() - start_time

        if elapsed < desired_time:
            data.ctrl[act_id] = desired_torque / gear_ratio
        elif desired_time < elapsed < desired_time + 2:
            data.ctrl[act_id] = 0.0
        else:
            pass 

        mujoco.mj_step(model, data)
        viewer.sync()
