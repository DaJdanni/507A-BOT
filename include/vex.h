#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>

#include "v5.h"
#include "v5_vcs.h"

#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <utility>
#include <bits/stdc++.h>
#include <memory>
#include <vector>
#include <functional>
#include <initializer_list>

// // these undefinitions are done in order to properly use eigen [https://www.vexforum.com/t/eigen-integration-issue/61474/6]
#undef __ARM_NEON__
#undef __ARM_NEON
#include "Eigen/Eigen"

#include "utility/util.hpp"
#include "utility/enums.hpp"
#include "utility/coordinates.hpp"
#include "utility/trackingWheel.hpp"
#include "main/MCL.hpp"
#include "main/pid.hpp"
#include "main/odometry.hpp"
#include "main/robot.hpp"
#include "utility/dataSink.hpp"
#include "utility/asset.hpp"
#include "autons.hpp"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)