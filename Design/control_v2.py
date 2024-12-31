import numpy as np
import time
import os

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

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
            bot.arm.go_to_home_pose()

            # Read xyz coordinates from output.txt and store them in a list of object poses
            object_poses = []
            with open('output.txt', 'r') as file:
                for line in file:
                    object_pose = list(map(float, line.strip().split(', ')))
                    object_poses.append(object_pose)

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

                # Move the end effector to the target position
                try:
                    # Link lengths and offsets

                    dx, dy, dz = target_pose

                    l1 = 89.45
                    l2 = 100
                    l21 = 35
                    l3 = 100
                    l4 = 129.15
                    # phi = -np.pi/2  # Angle in radians

                    

                    theta1 = np.arctan2(dy, dx)

                    phi = theta1  # Angle in radians

                    A = dx - l4 * np.cos(theta1) * np.cos(phi)
                    B = dy - l4 * np.sin(theta1) * np.cos(phi)
                    C = dz - l1 - l4 * np.sin(phi)

                    theta3 = np.arccos(((A*A + B*B + C*C) - (l2*l2) - (l3*l3)) / (2*l2*l3))

                    a = l3 * np.sin(theta3)
                    b = l2 + l3 * np.cos(theta3)
                    c = dz - l1 - l4 * np.sin(phi)
                    r = np.sqrt(a**2 + b**2)

                    theta2 = np.arctan2(c, np.sqrt(r**2 - c**2)) - np.arctan2(a, b)
                    theta4 = phi - theta2 - theta3



                    theta1_rad = np.radians(theta1)
                    theta2_rad = np.radians(theta2)
                    theta3_rad = np.radians(theta3)
                    theta4_rad = np.radians(theta4)

                    joint_positions = [theta1, theta2, theta3, theta4]

                    bot.arm.set_joint_positions(joint_positions)

                    time.sleep(1)

                    final_ee_pose = bot.arm.get_ee_pose()
                    print(f"Final end effector pose {i} (under base coordinate system):\n", np.round(final_ee_pose, 2))

                    # Wait for spraying
                    time.sleep(5)

                    bot.arm.go_to_home_pose()

                except (ValueError, ZeroDivisionError) as e:
                    print(f"Error occurred: {e}")

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