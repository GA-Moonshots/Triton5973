// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //---------------
    // -- PORTS  ---
    // --------------

    // CONTROLLERS
    public final static int BIGBOI = 2;
    public final static int XBOX = 1;

    // SWERVE MOTORS
    public final static int FRONT_RIGHT_SPEED_MOTOR = 0;
    public final static int FRONT_LEFT_SPEED_MOTOR = 1;
    public final static int BACK_RIGHT_SPEED_MOTOR = 1;
    public final static int BACK_LEFT_SPEED_MOTOR = 1;

    public final static int FRONT_RIGHT_ANGLE_MOTOR = 0;
    public final static int FRONT_LEFT_ANGLE_MOTOR = 1;
    public final static int BACK_RIGHT_ANGLE_MOTOR = 1;
    public final static int BACK_LEFT_ANGLE_MOTOR = 1;

    public final static int FRONT_RIGHT_ENCODER_A = 4;
    public final static int FRONT_RIGHT_ENCODER_B = 5;

    public final static int FRONT_LEFT_ENCODER_A = 7;
    public final static int FRONT_LEFT_ENCODER_B = 6;

    public final static int BACK_RIGHT_ENCODER_A = 3;
    public final static int BACK_RIGHT_ENCODER_B = 2;

    public final static int BACK_LEFT_ENCODER_A = 0;
    public final static int BACK_LEFT_ENCODER_B = 1;

    // AXIS
    

    // MECHANUM MOTORS
    public final static int MOTOR1 = 0;
    public final static int MOTOR2 = 1;
    public final static int MOTOR3 = 2;
    public final static int MOTOR4 = 3;

    // OTHER MOTOR PORTS
    public final static int BALL_THROWER_FRONT_WHEELS_PORT = 0;
    public final static int BALL_THROWER_BACK_WHEELS_PORT = 1;
    public final static int INTAKE_PORT = 2;

    // SENSOR PORTS
    public final static int RELAY_PORT = 1;
    public final static int ENCODER_PORT_1 = 2;
    public final static int ENCODER_PORT_2 = 3;

    // XBOX BUTTON IDS
    public final static int A = 1;
    public final static int B = 2;
    public final static int X = 3;
    public final static int Y = 4;
    public final static int L_BUMP = 5;
    public final static int R_BUMP = 6;
    public final static int SELECT = 7;
    public final static int START = 8;
    public final static int LEFT_JOYSTICK_BUTTON = 9;
    public final static int RIGHT_JOYSTICK_BUTTON = 10;
    public final static int LEFT_XBOX_AXIS = 0;


    //--------------------------
    // -- FORMULA CONSTANTS  ---
    // -------------------------

    public final static double TICKS_TO_DEGREES = 1.12;

    // SPEEDS
    public final static double ENCODER_SPEED = 0.3;
    public final static double DRIVE_TO_WALL_MAX = 0.6;
    public final static double DRIVE_TO_WALL_MIN = 0.3;
    public final static double GYRO_TURN_MAX = 0.7;
    public final static double GYRO_TURN_MIN = 0.35;
    public final static double DRIVE_FOR_TIME_SPEED = 0.5;

    // OTHER
    public final static int ENCODER_COUNTS_PER_REV = 2048;
    public final static int WHEEL_DIAMETER = 6;
    
    // How much time the relay stays on
    public final static double RELAY_STOP_TIME = 0.5;

    // BALL THROWER CONSTANTS
    public final static double BALL_THROWER_LARGE_WHEEL_SPEED = -1;
    public final static double BALL_THROWER_SMALL_WHEEL_SPEED = 1;
    public final static double BALL_THROWER_OFF = 0;

    // INTAKE THROWER CONSTANTS
    public final static double INTAKE_WHEEL_SPEED = 0.7;
    public final static double INTAKE_OFF_WHEEL_SPEED = 0;

}
