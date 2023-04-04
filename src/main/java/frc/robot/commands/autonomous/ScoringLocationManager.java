// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.utility.MirrorPoses.mirror;

public class ScoringLocationManager {
    private ScoringLocationManager() {}

    public enum ScoringLevel {
        HYBRID(.2, 0),
        MID(.6, Math.PI/5.),
        HIGH(1, Math.PI/3.);

        private final double motorSpeed;
        private final double clawAngleRadians;
        ScoringLevel(double motorSpeed, double clawAngleRadians) {
            this.motorSpeed = motorSpeed;
            this.clawAngleRadians = clawAngleRadians;
        }

        public double getRequiredMotorSpeed() {
            return motorSpeed;
        }
        
        public double getRequiredClawAngle() {
            return clawAngleRadians;
        }
    }

    private static final double DISTANCE_FROM_WALL = 2;

    public static Pose2d getNearestNodeAlignment() {
        final double currentPoseY = Drivetrain.getInstance().getPose2d().getY();
        final double nodeSpacing = Units.inchesToMeters(22);
        final double firstNodePosition = Units.inchesToMeters(20);
        final double offset = firstNodePosition - (nodeSpacing / 2.);

        final int nodeNumber = (int) ((currentPoseY - offset) / nodeSpacing);
        // Return if the robot's pose is too high
        if (nodeNumber > 8) {
            return Drivetrain.getInstance().getPose2d();
        }
        
        final double nearestPoseY = nodeNumber * nodeSpacing + (nodeSpacing / 2.) + offset;
        if (Robot.isRedAlliance()) {
            return mirror(new Pose2d(DISTANCE_FROM_WALL, nearestPoseY, Rotation2d.fromDegrees(180)));
        }
        return new Pose2d(DISTANCE_FROM_WALL, nearestPoseY, Rotation2d.fromDegrees(180));
    }

    public static Pose2d getNearestCubeNodeAlignment() {
        final double currentPoseY = Drivetrain.getInstance().getPose2d().getY();
        final double nodeSpacing = Units.inchesToMeters(66);
        final double firstNodePosition = Units.inchesToMeters(42);
        final double offset = firstNodePosition - (nodeSpacing / 2.);

        final int nodeNumber = (int) ((currentPoseY - offset) / nodeSpacing);
        // Return if the robot's pose is too high
        if (nodeNumber > 2) {
            return Drivetrain.getInstance().getPose2d();
        }
        
        final double nearestPoseY = nodeNumber * nodeSpacing + (nodeSpacing / 2.) + offset;
        if (Robot.isRedAlliance()) {
            return mirror(new Pose2d(DISTANCE_FROM_WALL, nearestPoseY, Rotation2d.fromDegrees(180)));
        }
        return new Pose2d(DISTANCE_FROM_WALL, nearestPoseY, Rotation2d.fromDegrees(180));
    }
}
