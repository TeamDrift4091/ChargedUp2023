package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static class Drivetrain {
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(4);
        public static final double DRIVETRAIN_GEAR_RATIO = 8.14; // L1 from SDS
        // Dimensions
        public static final double WHEEL_BASE_LENGTH_METERS = 1;
        public static final double WHEEL_BASE_WIDTH_METERS = 1;
        public static final double WHEEL_BASE_CENTER_POINT_METERS = WHEEL_BASE_LENGTH_METERS/2.; // Distance in meters from the back of the robot
        // CAN IDs
        public static final int FRONT_LEFT_DRIVE_CHANNEL = 1;
        public static final int FRONT_LEFT_STEER_CHANNEL = 2;
        public static final int FRONT_RIGHT_DRIVE_CHANNEL = 3;
        public static final int FRONT_RIGHT_STEER_CHANNEL = 4;
        public static final int BACK_LEFT_DRIVE_CHANNEL = 5;
        public static final int BACK_LEFT_STEER_CHANNEL = 6;
        public static final int BACK_RIGHT_DRIVE_CHANNEL = 7;
        public static final int BACK_RIGHT_STEER_CHANNEL = 8;
        public static final int FRONT_LEFT_CANCODER_CHANNEL = 9;
        public static final int FRONT_RIGHT_CANCODER_CHANNEL = 10;
        public static final int BACK_LEFT_CANCODER_CHANNEL = 11;
        public static final int BACK_RIGHT_CANCODER_CHANNEL = 12;
        // ENCODER OFFSETS
        public static final double FRONT_LEFT_ENCODER_OFFSET = 0;
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 0;
        public static final double BACK_LEFT_ENCODER_OFFSET = 0;
        public static final double BACK_RIGHT_ENCODER_OFFSET = 0;
    }
}