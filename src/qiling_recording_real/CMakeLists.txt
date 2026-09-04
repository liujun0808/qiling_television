cmake_minimum_required(VERSION 3.16)
project(qiling_recording_real)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/episode_recorder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME episode_recorder
)

install(PROGRAMS
  scripts/check_episode_quality.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME check_episode_quality
)

install(PROGRAMS
  scripts/build_training_admission_manifest.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME build_training_admission_manifest
)

install(PROGRAMS
  scripts/export_manifest_to_intermediate.py
  scripts/pack_intermediate_to_lerobot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_tri_cameras.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
