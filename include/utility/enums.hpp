#ifndef ENUMS_HPP
#define ENUMS_HPP

enum DTPConfig {DEFAULT, BOOMERANG, RAMSETE};
enum ODOMETRY_CONFIG {ZERO_TRACKER_ODOM, ONE_HORIZONTAL_TRACKER_ODOM, TWO_TRACKER_ODOM};
enum DATA_TARGETS 
{
    MOTOR,
    ROBOT,
    SENSOR
};
enum DATA_TYPES 
{
    // NOTE TO SELF: if you can find a better way please replace this mess
    // Motors:
    MOTOR_POSITION_DEG,
    MOTOR_POSITION_REV,
    MOTOR_POSITION_RAW,
    MOTOR_VELOCITY_PCT,
    MOTOR_VELOCITY_RPM,
    MOTOR_VELOCITY_DPS,
    MOTOR_TORQUE_NM,
    MOTOR_TEMPERATURE_CEL,
    MOTOR_TEMPERATURE_FAH,
    ODOMETRY_COORDINATE_X,
    ODOMETRY_COORDINATE_Y,
    ODOMETRY_COORDINATE_ROTATION,
    ODOMETRY_COORDINATE_X_REVERSED,
    ODOMETRY_COORDINATE_Y_REVERSED,
    ODOMETRY_COORDINATE_ROTATION_RAD,
    ROTATION_SENSOR_ANGLE,
    ROTATION_SENSOR_POSITION,
    ROTATION_SENSOR_VELOCITY,
    MOTOR_VOLTAGE
};

#endif