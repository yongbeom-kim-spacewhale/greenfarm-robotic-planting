from glob import glob

from setuptools import find_packages, setup


package_name = "robot_tests"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "robot_tests.integrated": ["smartfarm_dashboard.ui"],
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="spacewhale0107",
    maintainer_email="murderer0107@gmail.com",
    description="Test nodes for Doosan M0609 and peripherals.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "a_tray_second_drop_debug_node = robot_tests.standalone_tests.a_tray_second_drop_debug_node:main",
            "go_home_test = robot_tests.go_home_test_node:main",
            "integrated_dashboard = robot_tests.integrated.main:main",
            "integrated_emergency_stop_node = robot_tests.integrated_emergency_stop_node:main",
            "integrated_manual_control_node = robot_tests.integrated_manual_control_node:main",
            "integrated_tool_control_node = robot_tests.integrated_tool_control_node:main",
            "integrated_plant_pick_and_place_node = robot_tests.integrated_plant_pick_and_place_node:main",
            "integrated_tray_next_step_node = robot_tests.integrated_tray_next_step_node:main",
            "integrated_tray_soil_flatten_node = robot_tests.integrated_tray_soil_flatten_node:main",
            "abcd_tray_soil_drop_test_node = robot_tests.standalone_tests.abcd_tray_soil_drop_test_node:main",
            "keyboard_teleop_node = robot_tests.standalone_tests.keyboard_teleop_node:main",
            "plant_pick_and_place_tray_select_node = robot_tests.standalone_tests.plant_pick_and_place_tray_select_node:main",
            "press_plate_pickup_node = robot_tests.standalone_tests.press_plate_pickup_node:main",
            "press_plate_putdown_node = robot_tests.standalone_tests.press_plate_putdown_node:main",
            "shovel_pickup_node = robot_tests.standalone_tests.shovel_pickup_node:main",
            "shovel_putdown_node = robot_tests.standalone_tests.shovel_putdown_node:main",
            "soil_supply_scoop_test_node = robot_tests.standalone_tests.soil_supply_scoop_test_node:main",
            "simple_compliance_down_up_test = robot_tests.standalone_tests.simple_compliance_down_up_test:main",
            "tray_rock_remove_node = robot_tests.standalone_tests.tray_rock_remove_node:main",
            "tray_soil_add_node = robot_tests.standalone_tests.tray_soil_add_node:main",
            "tray_soil_keep_node = robot_tests.standalone_tests.tray_soil_keep_node:main",
            "tray_soil_flatten_node = robot_tests.standalone_tests.tray_soil_flatten_node:main",
            "tray_soil_remove_node = robot_tests.standalone_tests.tray_soil_remove_node:main",
            "tray_top_move_node = robot_tests.standalone_tests.tray_top_move_node:main",
            "tray_soil_state_check_node = robot_tests.standalone_tests.tray_soil_state_check_node:main",
            "integrated_tray_soil_state_check_node = robot_tests.integrated_tray_soil_state_check_node:main",
        ],
    },
)
