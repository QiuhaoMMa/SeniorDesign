from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

def main():

    # TODO: Define the joint angles in radians considering the joint limits
    joint_positions = [0.5,0.5 , 0.5, 0.5]

    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )
    bot.arm.go_to_home_pose()
    bot.arm.set_joint_positions(joint_positions)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

    bot.shutdown()


if __name__ == '__main__':
    main()