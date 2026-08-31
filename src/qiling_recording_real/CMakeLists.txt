cmake_minimum_required(VERSION 3.16)
project(qiling_recording_real)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/episode_recorder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME episode_recorder
)

install(PROGRAMS
  scripts/convert_to_lerobot.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME convert_to_lerobot
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
