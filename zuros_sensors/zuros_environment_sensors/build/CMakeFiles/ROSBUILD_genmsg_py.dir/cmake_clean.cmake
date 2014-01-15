FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/zuros_environment_sensors/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/zuros_environment_sensors/msg/__init__.py"
  "../src/zuros_environment_sensors/msg/_MSG_ZWAVE_STATUS.py"
  "../src/zuros_environment_sensors/msg/_MSG_ZWAVE_SENSORS.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
