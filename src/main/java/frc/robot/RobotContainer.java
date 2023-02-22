// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autonomous.AutonomousCommandManager;
import frc.robot.commands.drivetrain.AbsoluteAngleJoystickDrive;
import frc.robot.commands.drivetrain.DrivetrainTest;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.OrbitingJoystickDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.SmartHolonomicTrajectoryCommandGenerator;
import frc.team1891.common.control.JoystickRotation2d;

public class RobotContainer {
  // Subsystems
  Drivetrain drivetrain = Drivetrain.getInstance();
  Claw claw = new Claw();


  // Controllers
  XboxController controller = new XboxController(0) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
    };
  };

  // Buttons and triggers (These are how we schedule commands).

  JoystickRotation2d rightStickRotation = new JoystickRotation2d(() -> -controller.getRightY(), () -> -controller.getRightX());
  JoystickButton testDrivetrian = new JoystickButton(controller, 1);
  JoystickButton orbitDrive = new JoystickButton(controller, 2);
  JoystickButton squareAlign = new JoystickButton(controller, 3);

  JoystickButton toCommunity = new JoystickButton(controller, 4);
  JoystickButton toLoadingStation = new JoystickButton(controller, 5);

  public RobotContainer() {
    // Connects the buttons and triggers to commands
    configureBindings();
    // Loads the autonomous chooser with all of the available autonomous routines.
    AutonomousCommandManager.load();
  }

  private void configureBindings() {
    // Whenever not told to do something else, the drivetrian will run JoystickDrive.
    drivetrain.setDefaultCommand(
      new JoystickDrive(
        drivetrain,
        () -> controller.getLeftY(),
        () -> controller.getLeftX(),
        () -> controller.getRightX()
      )
      // new AbsoluteAngleJoystickDrive(
      //   drivetrain,
      //   () -> controller.getLeftY(), 
      //   () -> controller.getLeftX(),
      //   () -> rightStickRotation.get()
      // )
    );

    SmartDashboard.putData("Reset Odometry", new InstantCommand(() -> {
      // drivetrain.resetGyro();
      drivetrain.resetOdometry();
    }));

    testDrivetrian.onTrue(new DrivetrainTest(drivetrain));

    orbitDrive.whileTrue(new OrbitingJoystickDrive(drivetrain, () -> new Pose2d(4,4,new Rotation2d()), () -> controller.getLeftY(), () -> controller.getLeftX()));

    // squareAlign.whileTrue(AlignToAngle.alignToNearestSquare(drivetrain));
    squareAlign.whileTrue(new AbsoluteAngleJoystickDrive(drivetrain,
      () -> controller.getLeftY(),
      () -> controller.getLeftX(),
      () -> rightStickRotation.get()
    ));

    // balance.whileTrue(new BalanceOnChargingStation(drivetrain));
    toCommunity.whileTrue(SmartHolonomicTrajectoryCommandGenerator.toCommunityZone(drivetrain));
    toLoadingStation.whileTrue(SmartHolonomicTrajectoryCommandGenerator.toLoadingStation(drivetrain));
  }

  // This method runs at the beginning of the match to determine what command runs in autonomous.
  public Command getAutonomousCommand() {
    // Post to SmartDashboard that the command has started
    SmartDashboard.putBoolean("Autonomous Finished", false);
    // We need this in order to avoid a crash when running the same command twice. In a match this would never happen
    // but it's necessary for testing.
    CommandScheduler.getInstance().clearComposedCommands();
    // Returns the selected command from AutonomousCommandManager and appends a simple instant command that tells
    // SmartDashboard the command finished.
    return AutonomousCommandManager.getSelected().andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
  } 
}
