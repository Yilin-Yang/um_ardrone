add_library(lib${PROJECT_NAME}
  ardrone_to_robot_localization.cpp
  mag_imu_rebroadcaster.cpp
  navdata_altitude_rebroadcaster.cpp
  rebroadcaster.cpp
  templated_rebroadcaster.cpp
  tum_ekf_rebroadcaster.cpp
)
