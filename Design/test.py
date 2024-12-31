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

    # 获取睡眠时的末端执行器位置
    sleep_ee_pose = bot.arm.get_ee_pose()
    print("Sleep end effector pose (under base coordinate system):\n", sleep_ee_pose)
    translation1 = sleep_ee_pose[:3, 3]

    bot.arm.go_to_home_pose()

    # 获取家庭时的末端执行器位置
    home_ee_pose = bot.arm.get_ee_pose()
    print("Home end effector pose (under base coordinate system):\n", home_ee_pose)
    translation2 = home_ee_pose[:3, 3]

    print("Translation1:", translation1)
    print("Translation2:", translation2)

    # 移动末端执行器到指定位置
    target_pose = [0.1, 1.0, 1.0]

    bot.arm.set_ee_cartesian_trajectory(x=target_pose[0], y=target_pose[1], z=target_pose[2])
    ee_pose = bot.arm.get_ee_pose()
    translation3 = ee_pose[:3, 3]
    print("Target end effector pose (under base coordinate system):\n", ee_pose)
    print("Translation3:", translation3)

    time.sleep(2)

    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    main()
