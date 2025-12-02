cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)

ament_package()

install(PROGRAMS
  
  scripts/plot_slip_vs_sim_time.py

  scripts/plot_ppc.py

  scripts/uwb_data.py
  
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)
