// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.drivetrain.DriveToPose;
import frc.robot.subsystems.Drivetrain;
import frc.team1891.common.trajectory.HolonomicTrajectoryCommandGenerator;

import static frc.robot.utility.MirrorPoses.mirror;

/**
 * Helper class to hold our autonomous commands.
 * 
 * Many of our autonomous commands will be built by chaining other commands together in sequence and parallel.
 * 
 * All our commands will be posted at options on {@link SmartDashboard} using a {@link SendableChooser}.
 */
public class AutonomousCommandManager {
    private AutonomousCommandManager() {}

    private static SendableChooser<Pair<Command, Command>> commandChooser = new SendableChooser<>();

    public static void load() {
        // RotatingHolonomicDriveController.enableSmartDashboard(true);
        HolonomicTrajectoryCommandGenerator.setRotationalPID(DrivetrainConstants.rotationalP, DrivetrainConstants.rotationalI, DrivetrainConstants.rotationalD);
        HolonomicTrajectoryCommandGenerator.setTranslationalPID(DrivetrainConstants.translationalP, DrivetrainConstants.translationalI, DrivetrainConstants.translationalD);

        commandChooser.setDefaultOption("Default - Exit Community", new Pair<Command, Command>(
            new DriveToPose(Drivetrain.getInstance(), () -> new Pose2d(5, 5, Rotation2d.fromDegrees(190))),
            new DriveToPose(Drivetrain.getInstance(), () -> mirror(new Pose2d(5, 5, Rotation2d.fromDegrees(190))))
        ));

        commandChooser.addOption("Curve Trajectory", new Pair<Command, Command>(
            HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false,
                new Pair<Pose2d, Rotation2d>(new Pose2d(0,0, new Rotation2d()), Rotation2d.fromDegrees(0)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(1,2, new Rotation2d()), Rotation2d.fromDegrees(90)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(4,4, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(135))
            ),
            HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false,
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(0,0, new Rotation2d()), Rotation2d.fromDegrees(0))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(1,2, new Rotation2d()), Rotation2d.fromDegrees(90))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(4,4, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(135)))
            )
        ));

        // commandChooser.addOption("Curve Trajectory without Explicit Angle Control", HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false, false,
        //     mirror(new Pose2d(0,0, new Rotation2d())),
        //     mirror(new Pose2d(1,2, new Rotation2d())),
        //     mirror(new Pose2d(4,4, Rotation2d.fromDegrees(90)))
        // ).andThen(new DriveToPose(Drivetrain.getInstance(), () -> new Pose2d(4, 4, Rotation2d.fromDegrees(90))))
        // );

        commandChooser.addOption("Curve Trajectory 2", new Pair<Command, Command>(
            HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false, false,
                new Pair<Pose2d, Rotation2d>(new Pose2d(0,0, new Rotation2d()), Rotation2d.fromDegrees(0)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(3,2, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(90)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(4,4, Rotation2d.fromDegrees(110)), Rotation2d.fromDegrees(110)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(4,6, Rotation2d.fromDegrees(20)), Rotation2d.fromDegrees(20)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(7,3, Rotation2d.fromDegrees(-40)), Rotation2d.fromDegrees(140)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(8,1, Rotation2d.fromDegrees(-150)), Rotation2d.fromDegrees(0)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(5,1, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(0)))
                .andThen(new DriveToPose(Drivetrain.getInstance(), () -> new Pose2d(5, 1, Rotation2d.fromDegrees(0)))),
            HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false, false,
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(0,0, new Rotation2d()), Rotation2d.fromDegrees(0))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3,2, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(90))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(4,4, Rotation2d.fromDegrees(110)), Rotation2d.fromDegrees(110))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(4,6, Rotation2d.fromDegrees(20)), Rotation2d.fromDegrees(20))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(7,3, Rotation2d.fromDegrees(-40)), Rotation2d.fromDegrees(140))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(8,1, Rotation2d.fromDegrees(-150)), Rotation2d.fromDegrees(0))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(5,1, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(0))))
                .andThen(new DriveToPose(Drivetrain.getInstance(), () -> mirror(new Pose2d(5, 1, Rotation2d.fromDegrees(0)))))
        ));
        
        // commandChooser.addOption("Curve Trajectory 2 without Explicit Angle Control", HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false, false,
        //     mirror(new Pose2d(0,0, new Rotation2d())),
        //     mirror(new Pose2d(3,2, Rotation2d.fromDegrees(90))),
        //     mirror(new Pose2d(4,4, Rotation2d.fromDegrees(110))),
        //     mirror(new Pose2d(4,6, Rotation2d.fromDegrees(20))),
        //     mirror(new Pose2d(7,3, Rotation2d.fromDegrees(-40))),
        //     mirror(new Pose2d(8,1, Rotation2d.fromDegrees(-150))),
        //     mirror(new Pose2d(5,1, Rotation2d.fromDegrees(180)))
        // ).andThen(new DriveToPose(Drivetrain.getInstance(), () -> new Pose2d(5, 1, Rotation2d.fromDegrees(180))))
        // );

        commandChooser.addOption("Clover Test", new Pair<Command, Command>(
            HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false,
                new Pair<Pose2d, Rotation2d>(new Pose2d(2, 1, new Rotation2d(0)), new Rotation2d(Math.PI/2.)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(3, 2, new Rotation2d(Math.PI/2.)), new Rotation2d(Math.PI)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(2, 3, new Rotation2d(Math.PI)), new Rotation2d(3*Math.PI/2.)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(1, 2, new Rotation2d(3*Math.PI/2.)), new Rotation2d(0)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(2, 1, new Rotation2d(0)), new Rotation2d(Math.PI/2.)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(3, 2, new Rotation2d(Math.PI/2.)), new Rotation2d(Math.PI)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(2, 1, new Rotation2d(0)), new Rotation2d(Math.PI/2.)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(1, 2, new Rotation2d(3*Math.PI/2.)), new Rotation2d(0)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(2, 3, new Rotation2d(Math.PI)), new Rotation2d(3*Math.PI/2.)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(3, 2, new Rotation2d(Math.PI/2.)), new Rotation2d(Math.PI)),
                new Pair<Pose2d, Rotation2d>(new Pose2d(2, 1, new Rotation2d(0)), new Rotation2d(Math.PI/2.))
            ), HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false,
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(2, 1, new Rotation2d(0)), new Rotation2d(Math.PI/2.))), 
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3, 2, new Rotation2d(Math.PI/2.)), new Rotation2d(Math.PI))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(2, 3, new Rotation2d(Math.PI)), new Rotation2d(3*Math.PI/2.))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(1, 2, new Rotation2d(3*Math.PI/2.)), new Rotation2d(0))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(2, 1, new Rotation2d(0)), new Rotation2d(Math.PI/2.))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3, 2, new Rotation2d(Math.PI/2.)), new Rotation2d(Math.PI))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(2, 1, new Rotation2d(0)), new Rotation2d(Math.PI/2.))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(1, 2, new Rotation2d(3*Math.PI/2.)), new Rotation2d(0))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(2, 3, new Rotation2d(Math.PI)), new Rotation2d(3*Math.PI/2.))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(3, 2, new Rotation2d(Math.PI/2.)), new Rotation2d(Math.PI))),
                mirror(new Pair<Pose2d, Rotation2d>(new Pose2d(2, 1, new Rotation2d(0)), new Rotation2d(Math.PI/2.))))
        ));

        SmartDashboard.putData("Autonomous Chooser", commandChooser);
    }

    public static Command getSelected() {
        // SmartDashboard.putBoolean("Autonomous Finished", false);
        // return commandChooser.getSelected().andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            return commandChooser.getSelected().getFirst();
        }
        return commandChooser.getSelected().getSecond();
    }
}