import dm_control.mujoco
import mujoco.viewer
import time
import math


def main():
    try:
        m = dm_control.mujoco.MjModel.from_xml_path("worm.xml")
        d = dm_control.mujoco.MjData(m)

        motor_names = ["motor1", "motor2", "motor3", "motor4", "motor5", "motor6"]
        motor_ids = [dm_control.mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, m_name) for m_name in motor_names]

        with mujoco.viewer.launch_passive(m, d) as viewer:
            # Set camera parameters
            viewer.cam.azimuth = 200
            viewer.cam.elevation = -16
            viewer.cam.distance = 3.5
            viewer.cam.lookat[:] = [0.0, 0.0, 0.75]

            for t in range(1000):
                offset = 0
                for i in motor_ids:
                    d.ctrl[i] = 50*math.sin((t + offset*60) * 0.02)
                    offset+=1

                dm_control.mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(0.01)

            viewer.close()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()