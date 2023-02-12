// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Any poses or rotations given to the robot as target positions to drive to should be run through this first.
 */
public class MirrorPosesForRedAlliance {
    private MirrorPosesForRedAlliance() {}

    /**
     * If the robot is on the red alliance, the pose and rotation will be mirrored as if the robot were on the other alliance
     * @param poseAndHeading the {@link Pair}<{@link Pose2d},{@link Rotation2d}> that will be mirrored
     * @return the pose and heading that has been mirrored if necessary
     */
    public static Pair<Pose2d, Rotation2d> mirror(Pair<Pose2d, Rotation2d> poseAndHeading) {
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            return poseAndHeading;
        }
        return new Pair<Pose2d,Rotation2d>(
            mirror(poseAndHeading.getFirst()), 
            mirror(poseAndHeading.getSecond())
        );
    }

    /**
     * If the robot is on the red alliance, the pose will be mirrored as if the robot were on the other alliance
     * @param pose the {@link Pose2d} that will be mirrored
     * @return the pose that has been mirrored if necessary
     */
    public static Pose2d mirror(Pose2d pose) {
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            return pose;
        }
        return new Pose2d(Units.feetToMeters(54) - pose.getX(), pose.getY(), mirror(pose.getRotation()));
    }

    /**
     * If the robot is on the red alliance, the rotation will be mirrored as if the robot were on the other alliance
     * @param pose the {@link Rotation2d} that will be mirrored
     * @return the rotation that has been mirrored if necessary
     */
    public static Rotation2d mirror(Rotation2d rotation) {
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            return rotation;
        }
        return Rotation2d.fromDegrees(180).minus(rotation);
    }
}
