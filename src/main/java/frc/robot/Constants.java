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
        // public static final double DRIVETRAIN_STEER_GEAR_RATIO = 1;
        // Dimensions
        public static final double WHEEL_BASE_LENGTH_METERS = Units.inchesToMeters(30 - (1.75 * 2));
        public static final double WHEEL_BASE_WIDTH_METERS = Units.inchesToMeters(30 - (1.75 * 2));

        public static final double rotationalP = 4;
        public static final double rotationalI = 0.6;
        public static final double rotationalD = 0.5;

        public static final double translationalP = 2.5;
        public static final double translationalI = 0.5;
        public static final double translationalD = .4;

        public static final double CHASSIS_MAX_VELOCITY = 2;
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
            public static final double ENCODER_OFFSET_RADIANS = -Math.PI/2.;
        }
        public static class FrontRight {
            public static final int DRIVE_CHANNEL = 7;
            public static final int STEER_CHANNEL = 8;
            public static final double ENCODER_OFFSET_RADIANS = Math.PI-Math.PI/2.;
        }
        public static class BackLeft {
            public static final int DRIVE_CHANNEL = 1;
            public static final int STEER_CHANNEL = 2;
            public static final double ENCODER_OFFSET_RADIANS = -Math.PI/2.;
        }
        public static class BackRight {
            public static final int DRIVE_CHANNEL = 5;
            public static final int STEER_CHANNEL = 6;
            public static final double ENCODER_OFFSET_RADIANS = Math.PI-Math.PI/2.;
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
    public final class ArmConstants{
        // CAN IDs for three motors
        public static final int LEFT_CLIMBER_ID = 15; 
        public static final int RIGHT_CLIMBER_ID = 16;
        public static final int CLAW_STRING_ID = 17;
        public static final int SHOULDER_ID = 18;

        // Motor gear ratios
        public static final double SHOULDER_GEAR_RATIO = 1;
        public static final double ARM_GEAR_RATIO = 1;
        public static final double METERS_PER_CLIMBER_ROTATION = .05;

        // Joint height from ground 
        public static final double SHOULDER_HEIGHT_FROM_GROUND = 1.2;

        // Arm length limits
        public static final double ARM_MIN_LENGTH = .8;
        public static final double ARM_MAX_LENGTH = 1.8;

        public static final double ARM_MAX_ANGLE = 2*Math.PI/3.;
    }
    public static class ClawConstants {
        public static final int LEFT_CLAW_CHANNEL = 13;
        public static final int RIGHT_CLAW_CHANNEL = 14;
        public static final int PNEUMATIC_BACK_CHANNEL = 0;
        public static final int PNEUMATIC_FORWORD_CHANNEL = 1;
    }
}