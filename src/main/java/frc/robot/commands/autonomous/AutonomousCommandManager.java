// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.SmartHolonomicTrajectoryCommandGenerator;
import frc.team1891.common.trajectory.HolonomicTrajectoryCommandGenerator;

/**
 * Helper class to hold our autonomous commands.
 * 
 * Many of our autonomous commands will be built by chaining other commands together in sequence and parallel.
 * 
 * All our commands will be posted at options on {@link SmartDashboard} using a {@link SendableChooser}.
 */
public class AutonomousCommandManager {
    private AutonomousCommandManager() {}

    // This object appears on SmartDashboard, allowing us to select which autonomous routine we want, while the robot is on.
    private static SendableChooser<Command> commandChooser = new SendableChooser<>();

    /**
     * Loads the sendable chooser with all the autonomous options.
     */
    public static void load() {
        // RotatingHolonomicDriveController.enableSmartDashboard(true);
        HolonomicTrajectoryCommandGenerator.setRotationalPID(DrivetrainConstants.rotationalP, DrivetrainConstants.rotationalI, DrivetrainConstants.rotationalD);
        HolonomicTrajectoryCommandGenerator.setTranslationalPID(DrivetrainConstants.translationalP, DrivetrainConstants.translationalI, DrivetrainConstants.translationalD);

        // Default do nothing to avoid issues // TODO it's not default cause we have issus with Glass.
        commandChooser.addOption("Default - Do Nothing", new InstantCommand());

        // Drive forward for 3 seconds at roughly .5 m/s
        commandChooser.addOption("Drive Forward (~1.5 meters)",
            new RunCommand(() -> Drivetrain.getInstance().fromChassisSpeeds(new ChassisSpeeds(.5, 0, 0)), Drivetrain.getInstance()).withTimeout(3)
        );

        commandChooser.addOption("Backup then Forward",
            new SequentialCommandGroup(
                new RunCommand(() -> Drivetrain.getInstance().fromChassisSpeeds(new ChassisSpeeds(-.5, 0, 0)), Drivetrain.getInstance()).withTimeout(.5),
                new RunCommand(() -> Drivetrain.getInstance().fromChassisSpeeds(new ChassisSpeeds(.5, 0, 0)), Drivetrain.getInstance()).withTimeout(4)
        ));

        // Drive backward for 3 seconds at roughly .5 m/s
        commandChooser.addOption("Drive Backward (~1.5 meters)",
            new RunCommand(() -> Drivetrain.getInstance().fromChassisSpeeds(new ChassisSpeeds(-.5, 0, 0)), Drivetrain.getInstance()).withTimeout(3)
        );

        // commandChooser.setDefaultOption("Auto Charge", new AutoChargeStation(Drivetrain.getInstance()));
        commandChooser.setDefaultOption("Charge Station",
            new SequentialCommandGroup(
                new RunCommand(() -> Drivetrain.getInstance().fromChassisSpeeds(new ChassisSpeeds(-.5, 0, 0)), Drivetrain.getInstance()).withTimeout(.5),
                new RunCommand(() -> Drivetrain.getInstance().fromChassisSpeeds(new ChassisSpeeds(.5, 0, 0)), Drivetrain.getInstance()).withTimeout(2)
        ));

        // *******************************************************************************************************************
        // NOTE the robot must be placed so that the Limelight can see an AprilTag if you want to attempt anything besides the 
        // default command and the simple drive commands.
        // *******************************************************************************************************************

        // Leave the community without doing anything else
        // commandChooser.addOption("Leave Community", SmartHolonomicTrajectoryCommandGenerator.leaveCommunity(Drivetrain.getInstance()));

        // // Drive from current position to the nearest cone scoring location and place a cone on the ground
        // commandChooser.addOption("Score Cone - Low",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CONE, ScoringLevel.LOW)
        // );

        // // Drive from current position to the nearest cone scoring location and place a cone on the lower spike
        // commandChooser.addOption("Score Cone - Mid",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CONE, ScoringLevel.MEDIUM)
        // );

        // // Drive from current position to the nearest cone scoring location and place a cone on the highest spike
        // commandChooser.addOption("Score Cone - High",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CONE, ScoringLevel.HIGH)
        // );

        // // Drive from current position to the nearest cube scoring location and place a cube on the ground
        // commandChooser.addOption("Score Cube - Low",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CUBE, ScoringLevel.LOW)
        // );

        // // Drive from current position to the nearest cube scoring location and place a cube on the middle level
        // commandChooser.addOption("Score Cube - Mid",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CUBE, ScoringLevel.MEDIUM)
        // );

        // // Drive from current position to the nearest cube scoring location and place a cube on the highest level
        // commandChooser.addOption("Score Cube - High",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CUBE, ScoringLevel.HIGH)
        // );

        // Drive from current position to the nearest cone scoring location and place a cone on the ground and leave the community
        // commandChooser.addOption("Score Cone and Leave - Low",
            // new DriveToAndScore(Drivetrain.getInstance(), new Arm(), new Claw(), GameObject.CONE, ScoringLevel.LOW)
            //     .andThen(SmartHolonomicTrajectoryCommandGenerator.leaveCommunity(Drivetrain.getInstance()))
        // );

        // // Drive from current position to the nearest cone scoring location and place a cone on the lower spike and leave the community
        // commandChooser.addOption("Score Cone and Leave - Mid",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CONE, ScoringLevel.MEDIUM)
        //     .andThen(SmartHolonomicTrajectoryCommandGenerator.leaveCommunity(Drivetrain.getInstance()))
        // );

        // // Drive from current position to the nearest cone scoring location and place a cone on the highest spike and leave the community
        // commandChooser.addOption("Score Cone and Leave - High",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CONE, ScoringLevel.HIGH)
            //     .andThen(SmartHolonomicTrajectoryCommandGenerator.leaveCommunity(Drivetrain.getInstance()))
        // );

        // // Drive from current position to the nearest cube scoring location and place a cube on the ground and leave the community
        // commandChooser.addOption("Score Cube and Leave - Low",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CUBE, ScoringLevel.LOW)
            //     .andThen(SmartHolonomicTrajectoryCommandGenerator.leaveCommunity(Drivetrain.getInstance()))
        // );

        // // Drive from current position to the nearest cube scoring location and place a cube on the middle level and leave the community
        // commandChooser.addOption("Score Cube and Leave - Mid",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CUBE, ScoringLevel.MEDIUM)
            //     .andThen(SmartHolonomicTrajectoryCommandGenerator.leaveCommunity(Drivetrain.getInstance()))
        // );

        // // Drive from current position to the nearest cube scoring location and place a cube on the highest level and leave the community
        // commandChooser.addOption("Score Cube and Leave - High",
        //     new DriveToAndScore(Drivetrain.getInstance(), Arm.getInstance(), Claw.getInstance(), GameObject.CUBE, ScoringLevel.HIGH)
            //     .andThen(SmartHolonomicTrajectoryCommandGenerator.leaveCommunity(Drivetrain.getInstance()))
        // );

        SmartDashboard.putData("Autonomous Chooser", commandChooser);
    }

    /**
     * Gets the autonomous routine selected by the sendable chooser, and returns the correct command according to the alliance color.
     */
    public static Command getSelected() {
        // SmartDashboard.putBoolean("Autonomous Finished", false);
        // return commandChooser.getSelected().andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
        return commandChooser.getSelected();
    }
}