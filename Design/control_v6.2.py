import serial
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time
import os

def are_coordinates_equal(coord1, coord2, tolerance=0.05):
    return all(abs(c1 - c2) <= tolerance for c1, c2 in zip(coord1, coord2))

def main():

    ser = serial.Serial('/dev/ttyACM1', 9600)
  # Replace 'COMX' with the appropriate COM port
    time.sleep(5)


    processed_coordinates = set()  # Set to store processed coordinates

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
                        area = object_pose[3]
                        if area > 300:
                            area = 300
                        object_pose = [object_pose[0],object_pose[1],object_pose[2]]
                        is_unique = True
                        for processed_coord in processed_coordinates:
                            if are_coordinates_equal(processed_coord, object_pose):
                                is_unique = False
                                break
                        if is_unique:
                            object_poses.append(object_pose)
                            processed_coordinates.add(tuple(object_pose))
                    except ValueError:
                        print(f"Ignoring line in output.txt: {line.strip()}")

            for i, object_pose in enumerate(object_poses, 1):
                print(f"Object end effector pose {i} (under base coordinate system):\n", object_pose)

                object_r = np.sqrt(object_pose[0]**2 + object_pose[1]**2 + object_pose[2]**2)
                reach = 0.3
                spraydistance = 0.01*(np.sqrt(4*area/np.pi))/0.8
                print(f"disrance:\n", spraydistance)

                if object_r > (reach + spraydistance):
                    target_pose = [
                        reach * object_pose[0] / object_r,
                        reach * object_pose[1] / object_r,
                        reach * object_pose[2] / object_r
                    ]
                else:
                    target_pose = [
                        (reach - spraydistance) * object_pose[0] / object_r,
                        (reach - spraydistance) * object_pose[1] / object_r,
                        (reach - spraydistance) * object_pose[2] / object_r
                    ]

                print("Target end effector pose (under base coordinate system):\n", target_pose)
                print("Area:\n", area)



                r = np.sqrt(target_pose[0]**2 + target_pose[1]**2)

                bot.arm.set_single_joint_position(joint_name='waist', position=np.round(np.arctan2(target_pose[1], target_pose[0]), 3)) 

                bot.arm.set_ee_cartesian_trajectory(x=(-translation2[0] + r),z=(-translation2[2] + target_pose[2]))


                # finall_ee_pose = bot.arm.get_ee_pose()
                # print(f"End effector pose {i} (under base coordinate system):\n", np.round(finall_ee_pose, 2))

                time.sleep(6)

                final_ee_pose = bot.arm.get_ee_pose()
                print(f"Final end effector pose {i} (under base coordinate system):\n", np.round(final_ee_pose, 2))

                spray_ee_pose = bot.arm.get_ee_pose()
                
                spray_pose = spray_ee_pose[:3, 3]

                reference_pose = [object_pose[0] - spray_pose[0], object_pose[1] - spray_pose[1], object_pose[2] - spray_pose[2]] 
                

                alpha = -np.abs(np.round(np.arctan2(reference_pose[1], np.sqrt(reference_pose[0]**2 + reference_pose[1]**2)), 3))
                
                bot.arm.set_single_joint_position(joint_name='wrist_angle', position=1.5*alpha) 
                print()
                

                ser.write(b'S')  # Sending the signal to start spraying


                
                time.sleep(8)
                bot.arm.go_to_home_pose()
                time.sleep(3)

                # Calculate the target position and perform actions accordingly
                
                # Your actions for each object_pose go here...

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
