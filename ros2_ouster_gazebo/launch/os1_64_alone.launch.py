import launch
from launch import actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    # Declare launch arguments
    paused = DeclareLaunchArgument("paused", default_value="false")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    extra_gazebo_args = DeclareLaunchArgument("extra_gazebo_args", default_value="")
    gui = DeclareLaunchArgument("gui", default_value="true")
    recording = DeclareLaunchArgument("recording", default_value="false")
    headless = DeclareLaunchArgument("headless", default_value="false")
    debug = DeclareLaunchArgument("debug", default_value="false")
    physics = DeclareLaunchArgument("physics", default_value="ode")
    verbose = DeclareLaunchArgument("verbose", default_value="false")
    output = DeclareLaunchArgument("output", default_value="screen")
    world_name = DeclareLaunchArgument("world_name", default_value="$(find ros2_ouster_gazebo)/worlds/alone.world")
    respawn_gazebo = DeclareLaunchArgument("respawn_gazebo", default_value="false")
    use_clock_frequency = DeclareLaunchArgument("use_clock_frequency", default_value="false")
    pub_clock_frequency = DeclareLaunchArgument("pub_clock_frequency", default_value="100")
    robot_model = DeclareLaunchArgument("robot_model", default_value="$(find ros2_ouster_gazebo)/urdf/example_robot.urdf.xacro")

    # Set parameters
    set_sim_time_param = launch.actions.SetLaunchConfiguration("use_sim_time", LaunchConfiguration("use_sim_time"))

    # Prepare Gazebo arguments
    command_arg1 = launch.substitutions.LaunchConfiguration("command_arg1", default="")
    command_arg2 = launch.substitutions.LaunchConfiguration("command_arg2", default="")
    command_arg3 = launch.substitutions.LaunchConfiguration("command_arg3", default="")
    script_type = launch.substitutions.LaunchConfiguration("script_type", default="gzserver")

    # Start Gazebo server
    start_gazebo_server = Node(
        package="gazebo_ros",
        executable=script_type,
        name="gazebo",
        respawn=LaunchConfiguration("respawn_gazebo"),
        output=LaunchConfiguration("output"),
        arguments=[
            command_arg1,
            command_arg2,
            command_arg3,
            "-e",
            LaunchConfiguration("physics"),
            LaunchConfiguration("extra_gazebo_args"),
            LaunchConfiguration("world_name"),
        ],
        parameters=[{
            "gazebo": {
                "pub_clock_frequency": LaunchConfiguration("pub_clock_frequency")
            }
        }],
        condition=launch.conditions.IfCondition(LaunchConfiguration("use_clock_frequency"))
    )

    # Start Gazebo client
    start_gazebo_client = Node(
        package="gazebo_ros",
        executable="gzclient",
        name="gazebo_gui",
        respawn=False,
        output=LaunchConfiguration("output"),
        arguments=[command_arg3],
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui"))
    )

    # Spawner for the sensor/robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_model",
        name="spawn_robot",
        arguments=["-urdf", "-param", "robot_description", "-x", "0", "-y", "0", "-z", "0", "-model", "robot"],
        parameters=[{
            "robot_description": Command([
                "xacro",
                LaunchConfiguration("robot_model")
            ])
        }]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "publish_frequency": 30.0
        }]
    )

    # Create the launch description and specify the actions
    ld = launch.LaunchDescription()

    # Add the launch arguments to the launch description
    ld.add_action(paused)
    ld.add_action(use_sim_time)
    ld.add_action(extra_gazebo_args)
    ld.add_action(gui)
    ld.add_action(recording)
    ld.add_action(headless)
    ld.add_action(debug)
    ld.add_action(physics)
    ld.add_action(verbose)
    ld.add_action(output)
    ld.add_action(world_name)
    ld.add_action(respawn_gazebo)
    ld.add_action(use_clock_frequency)
    ld.add_action(pub_clock_frequency)
    ld.add_action(robot_model)

    # Add the set parameter action
    ld.add_action(set_sim_time_param)

    # Add the Gazebo actions
    ld.add_action(command_arg1)
    ld.add_action(command_arg2)
    ld.add_action(command_arg3)
    ld.add_action(script_type)
    ld.add_action(start_gazebo_server)
    ld.add_action(start_gazebo_client)

    # Add the robot spawner actions
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher)

    return ld