// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.GameObject;

import static frc.robot.utility.MirrorPoses.mirror;

public class ScoringLocationManager {
    private static final double DISTANCE_FROM_WALL = 2;
    private ScoringLocationManager() {}

    /** https://www.desmos.com/calculator/hmytultfgz */
    public enum ScoringLevel {
        LOW(1.191514, 0.254, 1.191514, 0.254),
        MEDIUM(0.79629, 0.851662, 0.79629, 1.1176),
        HIGH(0.363728, 1.1557, 0.363728, 1.4224);

        private final Translation2d cube, cone;

        private ScoringLevel(double xCube, double yCube, double xCone, double yCone) {
            this.cube = new Translation2d(xCube, yCube);
            this.cone = new Translation2d(xCone, yCone);
        }

        public Translation2d getPoseFromWall(GameObject gameObject) {
            if (gameObject.equals(GameObject.CONE)) {
                return cone;
            }
            return cube;
        }
    }

    public enum Grid {
        BY_LOADING_ZONE(2),
        COOPERTITION(1),
        BY_WALL(0);

        public static Grid[] allGrids = new Grid[] {
            Grid.BY_LOADING_ZONE,
            Grid.COOPERTITION,
            Grid.BY_WALL
        };

        private final Translation2d centerPose;

        private Grid(int gridNumber) {
            // centerPose = new Translation2d(DISTANCE_FROM_WALL, gridNumber*(1.6764)+1.069975);
            centerPose = new Translation2d(DISTANCE_FROM_WALL, gridNumber*Units.inchesToMeters(66)+Units.inchesToMeters(42));
        }

        public Translation2d getRequiredDrivetrainPose() {
            return centerPose;
        }
    }

    public enum GridSection {
        // CONE1(-0.569975),
        // CUBE(0),
        // CONE2(0.569975);
        CONE1(-Units.inchesToMeters(22)),
        CUBE(0),
        CONE2(Units.inchesToMeters(22));

        public static GridSection[] allCones = new GridSection[] {
            GridSection.CONE1,
            GridSection.CONE2
        };

        private final Translation2d offsetFromGridCenter;

        private GridSection(double yOffset) {
            offsetFromGridCenter = new Translation2d(0, yOffset);
        }

        public Translation2d getRequiredDrivetrainPose(Grid grid) {
            return grid.centerPose.plus(offsetFromGridCenter);
        }
    }

    public static Pose2d getNearestScoringPosition(GameObject gameObject) {
        Drivetrain drivetrain = Drivetrain.getInstance();
        Translation2d currentPos = drivetrain.getPose2d().getTranslation();

        // Find nearest Grid
        Grid nearestGrid = Grid.COOPERTITION;
        double nearestDistance = Double.POSITIVE_INFINITY;
        for (Grid grid : Grid.allGrids) {
            double dist = currentPos.getDistance(grid.centerPose);
            if (dist < nearestDistance) {
                nearestDistance = dist;
                nearestGrid = grid;
            }
        }

        if (gameObject.equals(GameObject.CUBE)) {
            if (Robot.isRedAlliance()) {
                return mirror(new Pose2d(nearestGrid.getRequiredDrivetrainPose(), Rotation2d.fromDegrees(180)));
            }
            return new Pose2d(nearestGrid.getRequiredDrivetrainPose(), Rotation2d.fromDegrees(180));
        } else {
            // Find nearest cone Section within Grid
            GridSection nearestSection = null;
            double nearestSectionDistance = Double.POSITIVE_INFINITY;
            for (GridSection section : GridSection.allCones) {
                double dist = currentPos.getDistance(section.getRequiredDrivetrainPose(nearestGrid));
                if (dist < nearestSectionDistance) {
                    nearestSectionDistance = dist;
                    nearestSection = section;
                }
            }
            if (Robot.isRedAlliance()) {
                return mirror(new Pose2d(nearestSection.getRequiredDrivetrainPose(nearestGrid), Rotation2d.fromDegrees(180)));
            }
            return new Pose2d(nearestSection.getRequiredDrivetrainPose(nearestGrid), Rotation2d.fromDegrees(180));
        }
    }

    public static Translation2d getArmPosition(GameObject gameObject, ScoringLevel scoringLevel) {
        Translation2d poseFromWall = scoringLevel.getPoseFromWall(gameObject);
        return new Translation2d(DISTANCE_FROM_WALL-poseFromWall.getX(), poseFromWall.getY());
    }
}
