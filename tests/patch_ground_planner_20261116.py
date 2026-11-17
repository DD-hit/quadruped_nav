from pathlib import Path
root=Path('/home/rl/workspace/quadruped_nav');p=root/'ros2_ws/src/quadruped_navigation/src/global_planner.cpp';s=p.read_text();s=s.replace('declare_parameter<double>("robot_radius", 0.20);','declare_parameter<double>("robot_radius", 0.20);\n    declare_parameter<double>("minimum_support_fraction", 0.0);')
a='    // Collision check for vehicle body volume (same height and above),'
b='''    // A supported center is insufficient on a small pillar or box. Require
    // area support under the existing footprint; keep separate storeys intact.
    const double minimum_support = get_parameter("minimum_support_fraction").as_double();
    if (require_ground_support && minimum_support > 0.0) {
      int total = 0, supported = 0;
      for (int dx = -n; dx <= n; ++dx) {
        for (int dy = -n; dy <= n; ++dy) {
          if ((dx*dx + dy*dy)*r*r > radius_sq) continue;
          ++total;
          if (hasGroundSupport({idx.x+dx,idx.y+dy,idx.z}, false, 0,
              std::max(1,support_depth_cells))) ++supported;
        }
      }
      if (total == 0 || static_cast<double>(supported)/total < minimum_support) return false;
    }

'''+a
assert a in s;s=s.replace(a,b);p.write_text(s)
p=root/'ros2_ws/src/quadruped_navigation/config/navigation.yaml';s=p.read_text().replace('    robot_radius: 0.43','    robot_radius: 0.43\n    # Require area support, not just one occupied voxel under the center.\n    minimum_support_fraction: 0.8');p.write_text(s)
