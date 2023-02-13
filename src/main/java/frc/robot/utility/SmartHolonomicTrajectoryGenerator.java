// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Drivetrain;
import frc.team1891.common.trajectory.HolonomicTrajectory;
import frc.team1891.common.trajectory.HolonomicTrajectoryGenerator;
import frc.team1891.common.trajectory.HolonomicTrajectoryParameterizer.TrajectoryGenerationException;

import static frc.robot.utility.MirrorPosesForRedAlliance.mirror;

/** Generates an optimized trajectory to the desired end point based on the robot's current position. */
public class SmartHolonomicTrajectoryGenerator {
    public static HolonomicTrajectory toCommunityZone(Drivetrain drivetrain) {
        Pose2d robotPose = drivetrain.getPose2d();
        ArrayList<Pair<Pose2d, Rotation2d>> posesAndHeadings;
        if (robotPose.getY() < Units.feetToMeters(10)) {
            posesAndHeadings = new ArrayList<Pair<Pose2d, Rotation2d>>(List.of(
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(4.5, 1, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3, 1, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)))
            ));

            if (robotPose.getX() < Units.feetToMeters(10) && DriverStation.getAlliance().equals(Alliance.Blue) ||
            robotPose.getX() > Units.feetToMeters(54- 10) && DriverStation.getAlliance().equals(Alliance.Red)) {
                posesAndHeadings.remove(0);
            }
        } else {
            posesAndHeadings = new ArrayList<Pair<Pose2d, Rotation2d>>(List.of(
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(10, 7, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(4.5, 4.5, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3, 4.5, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180)))
            ));
            
            if (robotPose.getX() < Units.feetToMeters(40) && DriverStation.getAlliance().equals(Alliance.Blue) ||
            robotPose.getX() > Units.feetToMeters(54- 40) && DriverStation.getAlliance().equals(Alliance.Red)) {
                posesAndHeadings.remove(0);
                if (robotPose.getX() < Units.feetToMeters(10) && DriverStation.getAlliance().equals(Alliance.Blue) ||
                robotPose.getX() > Units.feetToMeters(54- 10) && DriverStation.getAlliance().equals(Alliance.Red)) {
                    posesAndHeadings.remove(0);
                }
            }
        }

        if (posesAndHeadings.get(0).getFirst().relativeTo(robotPose).getTranslation().getNorm() < .5 && posesAndHeadings.size() == 1) {
            throw new TrajectoryGenerationException("This trajectory is too small, and will likely result in a malformed spline.");
        }

        Pose2d robotNoRotationPose = new Pose2d(robotPose.getTranslation(), new Rotation2d());
        posesAndHeadings.add(0, new Pair<Pose2d,Rotation2d>(new Pose2d(robotPose.getTranslation(), posesAndHeadings.get(0).getFirst().relativeTo(robotNoRotationPose).getTranslation().getAngle()), robotPose.getRotation()));

        Pose2d[] poses = new Pose2d[posesAndHeadings.size()];
        Rotation2d[] rotations = new Rotation2d[posesAndHeadings.size()];
        for (int i = 0; i < posesAndHeadings.size(); i++) {
            poses[i] = posesAndHeadings.get(i).getFirst();
            rotations[i] = posesAndHeadings.get(i).getSecond();
        }

        ArrayList<Pose2d> points = new ArrayList<>();
        Collections.addAll(points, poses);
        ArrayList<Rotation2d> headings = new ArrayList<>();
        Collections.addAll(headings, rotations);


        TrajectoryConfig config = new TrajectoryConfig(
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond,
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared
        ).setKinematics(drivetrain.getKinematics());

        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            points,
            headings,
            config
        );

        return trajectory;
    }

    public static HolonomicTrajectory toLoadingStation(Drivetrain drivetrain) {
        Pose2d robotPose = drivetrain.getPose2d();
        ArrayList<Pair<Pose2d, Rotation2d>> posesAndHeadings;
        posesAndHeadings = new ArrayList<Pair<Pose2d, Rotation2d>>(List.of(
            mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(10, 7, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0))), // Waypoint necessary to avoid collisions from anywhere in the center of the field to the final point
            mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(Units.feetToMeters(54)-3, 7, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0))) // Final point - right in front of the loading station
        ));

        // Checks if the robot is in the community
        if (robotPose.getX() < Units.feetToMeters(15) && DriverStation.getAlliance().equals(Alliance.Blue) ||
        robotPose.getX() > Units.feetToMeters(54- 15) && DriverStation.getAlliance().equals(Alliance.Red)) {
            if (robotPose.getY() < Units.feetToMeters(5)) { // easier to exit community from the bottom
                posesAndHeadings.add(0, mirror(new Pair<Pose2d,Rotation2d>(new Pose2d(4.5, 1, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0))));
            } else { // easier to exit community from the top
                posesAndHeadings.add(0, mirror(new Pair<Pose2d,Rotation2d>(new Pose2d(3, 4.5, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0))));
                posesAndHeadings.add(1, mirror(new Pair<Pose2d,Rotation2d>(new Pose2d(4, 4.5, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0))));
            }
        // Checks if the robot is the robot is somewhere in the middle section where the second to last waypoint isn't necessary
        } else if ((robotPose.getX() > Units.feetToMeters(54) - 10 && DriverStation.getAlliance().equals(Alliance.Blue) ||
        robotPose.getX() < 10 && DriverStation.getAlliance().equals(Alliance.Red)) &&
        robotPose.getY() > 5) {
            posesAndHeadings.remove(0);
        }

        if (posesAndHeadings.get(0).getFirst().relativeTo(robotPose).getTranslation().getNorm() < .5 && posesAndHeadings.size() == 1) {
            throw new TrajectoryGenerationException("This trajectory is too small, and will likely result in a malformed spline.");
        }

        Pose2d robotNoRotationPose = new Pose2d(robotPose.getTranslation(), new Rotation2d());
        posesAndHeadings.add(0, new Pair<Pose2d,Rotation2d>(new Pose2d(robotPose.getTranslation(), posesAndHeadings.get(0).getFirst().relativeTo(robotNoRotationPose).getTranslation().getAngle()), robotPose.getRotation()));

        Pose2d[] poses = new Pose2d[posesAndHeadings.size()];
        Rotation2d[] rotations = new Rotation2d[posesAndHeadings.size()];
        for (int i = 0; i < posesAndHeadings.size(); i++) {
            poses[i] = posesAndHeadings.get(i).getFirst();
            rotations[i] = posesAndHeadings.get(i).getSecond();
        }

        ArrayList<Pose2d> points = new ArrayList<>();
        Collections.addAll(points, poses);
        ArrayList<Rotation2d> headings = new ArrayList<>();
        Collections.addAll(headings, rotations);


        TrajectoryConfig config = new TrajectoryConfig(
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond,
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared
        ).setKinematics(drivetrain.getKinematics());

        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            points,
            headings,
            config
        );

        return trajectory;
    }
}
