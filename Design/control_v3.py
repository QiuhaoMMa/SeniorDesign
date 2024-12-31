from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time
import os

def main():
    while True:
        # Check if output.txt file exists and is not empty
        if os.path.isfile('output.txt') and os.path.getsize('output.txt') > 0:
            # Continue with robot arm operations
            bot = InterbotixManipulatorXS(
                robot_model='px100',
                group_name='arm',
                gripper_name='gripper'
            )

            bot.arm.go_to_sleep_pose()

            # Get the end effector position when the robot is in sleep pose
            sleep_ee_pose = bot.arm.get_ee_pose()
            translation1 = sleep_ee_pose[:3, 3]

            bot.arm.go_to_home_pose()

            # Get the end effector position when the robot is in home pose
            home_ee_pose = bot.arm.get_ee_pose()
            translation2 = home_ee_pose[:3, 3]

            # Read xyz coordinates from output.txt and store them in a list of object poses
            object_poses = []
            with open('output.txt', 'r') as file:
                for line in file:
                    try:
                        object_pose = list(map(float, line.strip().split(', ')))
                        object_poses.append(object_pose)
                    except ValueError:
                        print(f"Ignoring line in output.txt: {line.strip()}")

            for i, object_pose in enumerate(object_poses, 1):
                print(f"Object end effector pose {i} (under base coordinate system):\n", object_pose)

                # Calculate the target position
                object_r = np.sqrt(object_pose[0]**2 + object_pose[1]**2 + object_pose[2]**2)
                reach = 0.3

                if object_r > reach:
                    target_pose = [
                        reach * object_pose[0] / object_r,
                        reach * object_pose[1] / object_r,
                        reach * object_pose[2] / object_r
                    ]
                else:
                    target_pose = object_pose

                print("Target end effector pose (under base coordinate system):\n", target_pose)

                r = np.sqrt(target_pose[0]**2 + target_pose[1]**2)

                bot.arm.set_single_joint_position(joint_name='waist', position=np.round(np.arctan2(target_pose[1], target_pose[0]), 3)) 

                bot.arm.set_ee_cartesian_trajectory(x=(-translation2[0] + r),z=(-translation2[2] + target_pose[2]))

                time.sleep(6)

                final_ee_pose = bot.arm.get_ee_pose()
                print(f"Final end effector pose {i} (under base coordinate system):\n", np.round(final_ee_pose, 2))
                print()

                ## Waiting for spraying
                time.sleep(5)
                bot.arm.go_to_home_pose()
                time.sleep(3)

            bot.arm.go_to_home_pose()
            bot.arm.go_to_sleep_pose()

            # Clear output.txt after processing
            open('output.txt', 'w').close()

            # Exit the program after processing
            break
        else:
            # Wait for output.txt file to have content
            time.sleep(1)

if __name__ == '__main__':
    main()
