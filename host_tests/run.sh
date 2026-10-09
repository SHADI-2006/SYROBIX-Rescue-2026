#!/bin/sh
# Host-side tests (no hardware, no Arduino toolchain; needs g++ with C++17).
# Usage: unzip next to the firmware so the sources are in ../ , then ./run.sh
set -e
SRC=$(cd ../src && pwd)
echo "== LidarBank XSHUT/addressing simulation"
( cd lidar_sim && cp ../stub/Arduino.h ../stub/esp_attr.h . && mkdir -p freertos \
  && cp ../stub/freertos/*.h freertos/ \
  && g++ -std=gnu++17 -I. -I"$SRC" sim.cpp "$SRC/LidarBank.cpp" -o sim && ./sim )
echo "== Mission state machine + PCA9685 scenarios (real TaskHandler.cpp)"
( cd mission_sim && F="-std=gnu++17 -Dprivate=public -w -I. -I../stub -I$SRC" \
  && g++ $F sim_th.cpp stubs.cpp "$SRC/RobotDrivetrain.cpp" "$SRC/SensorsSystem.cpp" \
         "$SRC/LidarBank.cpp" "$SRC/LineArray.cpp" -o sim_th && ./sim_th )
