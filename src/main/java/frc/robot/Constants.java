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
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
        public static final double DRIVETRAIN_DRIVE_GEAR_RATIO = 5.50; // 12T REV MAXSwerve
        // public static final double DRIVETRAIN_STEER_GEAR_RATIO = 1;
        // Dimensions
        public static final double WHEEL_BASE_LENGTH_METERS = Units.inchesToMeters(30 - (1.75 * 2));
        public static final double WHEEL_BASE_WIDTH_METERS = Units.inchesToMeters(30 - (1.75 * 2));

        public static final double rotationalP = 1;
        public static final double rotationalI = 0;
        public static final double rotationalD = 0;

        public static final double translationalP = 1;
        public static final double translationalI = 0;
        public static final double translationalD = 0;

        public static final double CHASSIS_MAX_VELOCITY = 2;
        public static final double CHASSIS_MAX_ACCELERATION = 1;
        public static final double CHASSIS_MAX_ANGULAR_VELOCITY = Math.PI;
        public static final double CHASSIS_MAX_ANGULAR_ACCELERATION = Math.PI;
        public static final double MODULE_MAX_VELOCITY = 3.5; // Free speed max is ~4.11 for REV MAXSwerve

        public static class FrontLeft {
            public static final int DRIVE_CHANNEL = 1;
            public static final int STEER_CHANNEL = 2;
            public static final int CANCODER_CHANNEL = 9;
            public static final double ENCODER_OFFSET_RADIANS = 0;
        }
        public static class FrontRight {
            public static final int DRIVE_CHANNEL = 3;
            public static final int STEER_CHANNEL = 4;
            public static final int CANCODER_CHANNEL = 10;
            public static final double ENCODER_OFFSET_RADIANS = 0;
        }
        public static class BackLeft {
            public static final int DRIVE_CHANNEL = 5;
            public static final int STEER_CHANNEL = 6;
            public static final int CANCODER_CHANNEL = 11;
            public static final double ENCODER_OFFSET_RADIANS = 0;
        }
        public static class BackRight {
            public static final int DRIVE_CHANNEL = 7;
            public static final int STEER_CHANNEL = 8;
            public static final int CANCODER_CHANNEL = 12;
            public static final double ENCODER_OFFSET_RADIANS = 0;
        }
    }
    public static class VisionConstants {
        public static final String CAMERA_NAME = "Main (Limelight)";
        // TODO: Physical location of the camera on the robot, relative to the center of the robot.
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
            new Translation3d(0, 0, 0), // in meters
            new Rotation3d()
        );
    }
    public final class ArmConstants{
        public static final int LEFT_CLIMBER_ID = 1; 
        public static final int RIGHT_CLIMBER_ID = 2;
        public static final int CLAW_STRING_ID = 3;
        //CAN IDs for three motors

        public static final double JOINT_HEIGHT_FROM_GROUND =1;
        //joint height from ground 

        public static final double ARM_MIN_LENGTH = 0.5;
        public static final double ARM_MAX_LENGTH = 1;
        //Arm length limits
    }
    public static class ClawConstants {
        public static final int LEFT_CLAW_CHANNEL = 13;
        public static final int RIGHT_CLAW_CHANNEL = 14;
        public static final int PNEUMATIC_BACK_CHANNEL = 0;
        public static final int PNEUMATIC_FORWORD_CHANNEL = 1;
    }
}