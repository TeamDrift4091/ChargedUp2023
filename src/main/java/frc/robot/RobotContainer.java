// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.AlignToAngle;
import frc.robot.commands.drivetrain.DrivetrainTest;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.OrbitingJoystickDrive;
import frc.robot.subsystems.Drivetrain;
import frc.team1891.common.control.JoystickRotation2d;

public class RobotContainer {
  // Subsystems
  Drivetrain drivetrain = Drivetrain.getInstance();


  // Controllers
  XboxController controller = new XboxController(0) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1);
    };
  };

  JoystickRotation2d rightStickRotation = new JoystickRotation2d(() -> -controller.getRightY(), () -> -controller.getRightX());
  JoystickButton testDrivetrian = new JoystickButton(controller, 1);
  JoystickButton orbitDrive = new JoystickButton(controller, 2);
  JoystickButton squareAlign = new JoystickButton(controller, 3);

  

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
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

    testDrivetrian.onTrue(new DrivetrainTest(drivetrain));

    orbitDrive.whileTrue(new OrbitingJoystickDrive(drivetrain, () -> new Pose2d(4,4,new Rotation2d()), () -> controller.getLeftY(), () -> controller.getLeftX()));

    squareAlign.whileTrue(AlignToAngle.alignToNearestSquare(drivetrain));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
