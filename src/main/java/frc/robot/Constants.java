package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class Drivetrain {
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
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
        // TODO: Tune
        public static final double FRONT_LEFT_ENCODER_OFFSET_RADIANS = 0;
        public static final double FRONT_RIGHT_ENCODER_OFFSET_RADIANS = 0;
        public static final double BACK_LEFT_ENCODER_OFFSET_RADIANS = 0;
        public static final double BACK_RIGHT_ENCODER_OFFSET_RADIANS = 0;
    }
    public static class Vision {
        public static final String CAMERA_NAME = "Main (Limelight)";
        // TODO: Physical location of the camera on the robot, relative to the center of the robot.
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
            new Translation3d(0, 0, 0), // in meters
            new Rotation3d()
        );
    }
}