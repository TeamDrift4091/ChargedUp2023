package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Units are in meters and radians
 */
public class Constants {
    public static double FIELD_WIDTH = Units.feetToMeters(27);
    public static double FIELD_LENGTH = Units.feetToMeters(54);
    public static class DrivetrainConstants {
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);
        public static final double DRIVETRAIN_DRIVE_GEAR_RATIO = 4.71; // 14T REV MAXSwerve
        public static final double DRIVETRAIN_STEER_GEAR_RATIO = 1; // TODO: Find actual gear ratio
        // Dimensions
        public static final double WHEEL_BASE_LENGTH_METERS = Units.inchesToMeters(30 - (1.75 * 2));
        public static final double WHEEL_BASE_WIDTH_METERS = Units.inchesToMeters(30 - (1.75 * 2));

        public static final double rotationalP = 4;
        public static final double rotationalI = 0.6;
        public static final double rotationalD = 0.5;

        public static final double rotationalPForHolonomic = 4.5;
        public static final double rotationalIForHolonomic = 0;
        public static final double rotationalDForHolonomic = 0;

        public static final double translationalP = 2.5;
        public static final double translationalI = 0.5;
        public static final double translationalD = .4;

        public static final double CHASSIS_MAX_VELOCITY = 2.75;
        public static final double CHASSIS_MAX_ACCELERATION = 1;
        public static final double CHASSIS_MAX_ANGULAR_VELOCITY = 2*Math.PI/3.;
        public static final double CHASSIS_MAX_ANGULAR_ACCELERATION = Math.PI;
        public static final double MODULE_MAX_VELOCITY = 3.5; // Free speed max is ~4.11 for REV MAXSwerve

        public static final double driveP = 0.02;
        public static final double driveI = 0.0;
        public static final double driveD = 0.0;
        public static final double driveF = 0.045;

        public static final double steerP = 1.0;
        public static final double steerI = 0.0;
        public static final double steerD = 0.0;
        public static final double steerF = 0.0;

        public static class FrontLeft {
            public static final int DRIVE_CHANNEL = 3;
            public static final int STEER_CHANNEL = 4;
            public static final int CANCODER_CHANNEL = 10;
            public static final double ENCODER_OFFSET_RADIANS = 0;
        }
        public static class FrontRight {
            public static final int DRIVE_CHANNEL = 7;
            public static final int STEER_CHANNEL = 8;
            public static final int CANCODER_CHANNEL = 12;
            public static final double ENCODER_OFFSET_RADIANS = 0;
        }
        public static class BackLeft {
            public static final int DRIVE_CHANNEL = 1;
            public static final int STEER_CHANNEL = 2;
            public static final int CANCODER_CHANNEL = 9;
            public static final double ENCODER_OFFSET_RADIANS = 0;
        }
        public static class BackRight {
            public static final int DRIVE_CHANNEL = 5;
            public static final int STEER_CHANNEL = 6;
            public static final int CANCODER_CHANNEL = 11;
            public static final double ENCODER_OFFSET_RADIANS = 0;
        }
    }
    public static class VisionConstants {
        public static final String CAMERA_NAME = "Limelight";
        // TODO: Physical location of the camera on the robot, relative to the center of the robot.
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
            new Translation3d(0, 0, 0), // in meters
            new Rotation3d()
        );
    }
}