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
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5969; // FIXEDME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5969; // FIXEDME Measure and set wheelbase

    public static final int DRIVETRAIN_NAVX_ID = 0; // FIXEDME Set Pigeon ID

    public static final int FALCON_UNITS_PER_ROTATION = 2048;

    /* Correct values
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXEDME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 9; // FIXEDME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 8; // FIXEDME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 1899;//-Math.toRadians(166.904296875); // FIXEDME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10; // FIXEDME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12; // FIXEDME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11; // FIXEDME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 3473;//-Math.toRadians(305.244140625 + 180.0); // FIXEDME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXEDME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXEDME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 5; // FIXEDME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 3976;//-Math.toRadians(349.453125); // FIXEDME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXEDME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3; // FIXEDME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2; // FIXEDME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 1177;//-Math.toRadians(103.447265625 + 180.0); // FIXEDME Measure and set back right steer offset
    */
    
    static enum SwerveModuleConstants
    {
        frontLeft(7, true, 8, 9, 1899){},
        frontRight(10, false, 11, 12, 3473){},
        backLeft(4, true, 5, 6, 3976){},
        backRight(1, false, 2, 3, 1177){};

        int driveMotorChannel;
        boolean driveMotorInverted;
        int turningMotorEncoder;
        int turningMotorEncoderOffset;
        int turningMotorChannel;

        /**
         * @param driveMotorChannel
         * @param driveMotorInverted
         * @param turningMotorEncoder
         * @param turningMotorEncoderOffset
         * @param turningMotorChannel
         */
        private SwerveModuleConstants(int driveMotorChannel, boolean driveMotorInverted, int turningMotorEncoder, int turningMotorEncoderOffset, int turningMotorChannel)
        {
            this.driveMotorChannel = driveMotorChannel;
            this.driveMotorInverted = driveMotorInverted;
            this.turningMotorEncoder = turningMotorEncoder;
            this.turningMotorEncoderOffset = turningMotorEncoderOffset;
            this.turningMotorChannel = turningMotorChannel;
        }

    }
}