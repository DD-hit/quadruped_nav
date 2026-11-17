from pathlib import Path
r=Path('/home/rl/workspace/quadruped_nav/ros2_ws/src')
p=r/'quadruped_interfaces/srv/GetNavigationMapMeta.srv';s=p.read_text().replace('float64 robot_radius','float64 minimum_support_fraction\nfloat64 robot_radius');p.write_text(s)
p=r/'quadruped_navigation/src/global_planner.cpp';s=p.read_text().replace('    response->robot_radius =', '    response->minimum_support_fraction = get_parameter("minimum_support_fraction").as_double();\n    response->robot_radius =');p.write_text(s)
p=r/'quadruped_navigation/quadruped_navigation_tools/map_package_manager.py';s=p.read_text().replace('"robot_radius": meta.robot_radius,','"robot_radius": meta.robot_radius,\n                "minimum_support_fraction": meta.minimum_support_fraction,');p.write_text(s)
