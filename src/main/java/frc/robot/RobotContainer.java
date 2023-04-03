// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.AutonomousCommandManager;
import frc.robot.commands.drivetrain.*;
import frc.robot.subsystems.*;

import static frc.robot.utility.MirrorPoses.mirror;

public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  // Controllers
  private final XboxController xboxController = new XboxController(0) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
    };
  };
 
  private Trigger resetOdometry;

  public RobotContainer() {
    // Connects the buttons and triggers to commands
    configureBindings();
    // Loads the autonomous chooser with all of the available autonomous routines.
    // I'm doing this on a seperate thread because loading trajectories can take a lot of time.
    Thread loadAutoThread = new Thread(() -> {
      long start = System.currentTimeMillis();
      AutonomousCommandManager.load();
      System.out.printf("AUTO: Autonomous commands loaded in %.3f seconds.\n", (System.currentTimeMillis() - start)/1000.);
    }, "AutonomousCommandManager.load();");
    loadAutoThread.run();
  }

  private void configureBindings() {
    // DEFAULT COMMANDS
    // Whenever not told to do something else, the drivetrian will run JoystickDrive.
    drivetrain.setDefaultCommand(
      new JoystickDrive(
        drivetrain,
        () -> xboxController.getLeftY(),
        () -> xboxController.getLeftX(),
        () -> xboxController.getRightX()
      )
    );

    resetOdometry.onTrue(new InstantCommand(() -> {
      if (Robot.isBlueAlliance()) {
        drivetrain.resetOdometry();
      } else {
        drivetrain.resetOdometry(mirror(new Pose2d()));
      }
    }));
  }

  // This method runs at the beginning of the match to determine what command runs in autonomous.
  public Command getAutonomousCommand() {
    // Post to SmartDashboard that the command has started
    SmartDashboard.putBoolean("Autonomous Finished", false);
    // We need this in order to avoid a crash when running the same command twice. In a match this would never happen
    // but it's necessary for testing.  We only need it because were adding the .andThen with the SmartDashboard output.
    CommandScheduler.getInstance().clearComposedCommands();
    // Returns the selected command from AutonomousCommandManager and appends a simple instant command that tells
    // SmartDashboard the command finished.
    return AutonomousCommandManager.getSelected().andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
  } 
}
