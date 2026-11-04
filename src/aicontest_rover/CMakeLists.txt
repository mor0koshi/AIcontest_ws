cmake_minimum_required(VERSION 3.8)
project(aicontest_rover)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# コースの配置(config/map/course.yaml)から地図CSVを作るスクリプト
install(PROGRAMS scripts/make_map.py scripts/sim_realism.py scripts/wheel_odometry_wls.py scripts/random_course.py scripts/course_value.py scripts/pose_extrapolator.py scripts/chair_aligner.py scripts/zone_speed_limiter.py scripts/sim_campaign.py scripts/scan_chairs.py scripts/chair_detector.py scripts/test_chair_detection.py scripts/test_routes.py scripts/route_check.py DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
    state_graph
    urdf
)
