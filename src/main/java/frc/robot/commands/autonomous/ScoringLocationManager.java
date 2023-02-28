// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.GameObject;

import static frc.robot.utility.MirrorPoses.mirror;

/** Add your docs here. */
public class ScoringLocationManager {
    private ScoringLocationManager() {}

    // TODO: Take measurements and implement
    public enum ScoringLevel {
        LOW(.3,.1),
        MEDIUM(.5, .3),
        HIGH(.8, .5);

        // The pose of the arm relative to the robot
        private final Translation2d armPose;

        private ScoringLevel(double x, double y) {
            this.armPose = new Translation2d(x, y);
        }

        public Translation2d getRequiredArmPose() {
            return armPose;
        }
    }

    public enum Grid {
        BY_LOADING_ZONE(3),
        COOPERTITION(2),
        BY_WALL(1);

        public static Grid[] allGrids = new Grid[] {
            Grid.BY_LOADING_ZONE,
            Grid.COOPERTITION,
            Grid.BY_WALL
        };

        private final Translation2d centerPose;

        private Grid(int gridNumber) {
            centerPose = new Translation2d(2, gridNumber*(1.6764)+1.069975);
        }

        public Translation2d getRequiredDrivetrainPose() {
            return centerPose;
        }
    }

    public enum GridSection {
        CONE1(-0.569975),
        CUBE(0),
        CONE2(0.569975);

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

    // TODO: May need different arm positions for different GameObjects
    public static Translation2d getArmPosition(ScoringLevel scoringLevel) {
        return scoringLevel.getRequiredArmPose();
    }
}
