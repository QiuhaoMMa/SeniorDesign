from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time

def main():
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

    

    # Calculate the target position
    object_pose = [0.3, 0.3, 0.3]
    object_r = np.sqrt(object_pose[0]**2 + object_pose[1]**2 + object_pose[2]**2)

    # reach = np.sqrt(0.04067969**2 + 0**2 + 0.40659947**2)



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
    bot.arm.set_single_joint_position(joint_name='waist', position=np.round(np.arctan2(target_pose[1],target_pose[0]),5))
    ee_pose = bot.arm.get_ee_pose()
    translation3 = ee_pose[:3, 3]

    bot.arm.set_ee_cartesian_trajectory(x= (-translation3[0] + target_pose[0]),z= (-translation3[2] + target_pose[2]))

    time.sleep(1)

    final_ee_pose = bot.arm.get_ee_pose()
    print("Final end effector pose (under base coordinate system):\n", np.round(final_ee_pose,2))

    ## Waiting for spraying
    time.sleep(5)

    ## Finish spraying

    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()


if __name__ == '__main__':
    main()