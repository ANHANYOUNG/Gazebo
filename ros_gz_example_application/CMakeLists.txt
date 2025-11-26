cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)

ament_package()

install(PROGRAMS
  scripts/pure_pursuit_controller.py
  scripts/plot_slip_vs_sim_time.py
  scripts/kalman_filter.py
  scripts/imu.py
  scripts/cmd_vel_to_motor_cmd.py
  scripts/state_machine.py
  scripts/cmd_send.py
  scripts/plot_ppc.py
  scripts/plot_ppc_2.py
  scripts/uwb_data.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)
