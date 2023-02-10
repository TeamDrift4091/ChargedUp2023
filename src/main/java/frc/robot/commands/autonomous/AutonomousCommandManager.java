// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.DriveToPose;
import frc.robot.subsystems.Drivetrain;
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

    private static SendableChooser<Command> commandChooser = new SendableChooser<>();

    public static void load() {
        HolonomicTrajectoryCommandGenerator.setRotationalPID(Constants.Drivetrain.rotationalP, Constants.Drivetrain.rotationalI, Constants.Drivetrain.rotationalD);
        HolonomicTrajectoryCommandGenerator.setTranslationalPID(Constants.Drivetrain.translationalP, Constants.Drivetrain.translationalI, Constants.Drivetrain.translationalD);

        commandChooser.setDefaultOption("Default - Exit Community", new DriveToPose(Drivetrain.getInstance(), () -> new Pose2d(5, 5, Rotation2d.fromDegrees(190))));

        commandChooser.addOption("Curve Trajectory", HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false,
            new Pair<Pose2d, Rotation2d>(new Pose2d(0,0, new Rotation2d()), Rotation2d.fromDegrees(0)),
            new Pair<Pose2d, Rotation2d>(new Pose2d(1,2, new Rotation2d()), Rotation2d.fromDegrees(90)),
            new Pair<Pose2d, Rotation2d>(new Pose2d(4,4, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(135))
        ));

        commandChooser.addOption("Curve Trajectory without Explicit Angle Control", HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false, false,
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(1,2, new Rotation2d()),
            new Pose2d(4,4, Rotation2d.fromDegrees(90))
        ).andThen(new DriveToPose(Drivetrain.getInstance(), () -> new Pose2d(4, 4, Rotation2d.fromDegrees(90))))
        );

        commandChooser.addOption("Curve Trajectory 2", HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false, false,
             new Pair<Pose2d, Rotation2d>(new Pose2d(0,0, new Rotation2d()), Rotation2d.fromDegrees(0)),
             new Pair<Pose2d, Rotation2d>(new Pose2d(3,2, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(90)),
             new Pair<Pose2d, Rotation2d>(new Pose2d(4,4, Rotation2d.fromDegrees(110)), Rotation2d.fromDegrees(110)),
             new Pair<Pose2d, Rotation2d>(new Pose2d(4,6, Rotation2d.fromDegrees(20)), Rotation2d.fromDegrees(20)),
             new Pair<Pose2d, Rotation2d>(new Pose2d(7,3, Rotation2d.fromDegrees(-40)), Rotation2d.fromDegrees(140)),
             new Pair<Pose2d, Rotation2d>(new Pose2d(8,1, Rotation2d.fromDegrees(-150)), Rotation2d.fromDegrees(0)),
             new Pair<Pose2d, Rotation2d>(new Pose2d(5,1, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(0))
        ).andThen(new DriveToPose(Drivetrain.getInstance(), () -> new Pose2d(5, 1, Rotation2d.fromDegrees(0))))
        );
        
        commandChooser.addOption("Curve Trajectory 2 without Explicit Angle Control", HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false, false,
            new Pose2d(0,0, new Rotation2d()),
            new Pose2d(3,2, Rotation2d.fromDegrees(90)),
            new Pose2d(4,4, Rotation2d.fromDegrees(110)),
            new Pose2d(4,6, Rotation2d.fromDegrees(20)),
            new Pose2d(7,3, Rotation2d.fromDegrees(-40)),
            new Pose2d(8,1, Rotation2d.fromDegrees(-150)),
            new Pose2d(5,1, Rotation2d.fromDegrees(180))
        ).andThen(new DriveToPose(Drivetrain.getInstance(), () -> new Pose2d(5, 1, Rotation2d.fromDegrees(180))))
        );

        commandChooser.addOption("Clover Test", HolonomicTrajectoryCommandGenerator.generate(Drivetrain.getInstance(), false,
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
        ));

        SmartDashboard.putData("Autonomous Chooser", commandChooser);
    }

    public static Command getSelected() {
        // SmartDashboard.putBoolean("Autonomous Finished", false);
        // return commandChooser.getSelected().andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
        return commandChooser.getSelected();
    }
}