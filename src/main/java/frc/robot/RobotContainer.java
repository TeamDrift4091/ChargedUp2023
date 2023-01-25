// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.DrivetrainTest;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.drivetrain.OrbitingJoystickDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  // Subsystems
  Drivetrain drivetrain = Drivetrain.getInstance();

  // Controllers
  Joystick controller = new Joystick(0);

  JoystickButton testDrivetrian = new JoystickButton(controller, 1); // Button 1

  JoystickButton orbitDrive = new JoystickButton(controller, 2);
  

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
      new JoystickDrive(
        drivetrain,
        () -> controller.getY(),
        () -> controller.getX(),
        () -> controller.getZ()
      )
    );

    testDrivetrian.onTrue(new DrivetrainTest(drivetrain));

    orbitDrive.whileTrue(new OrbitingJoystickDrive(drivetrain, () -> new Pose2d(1,2,new Rotation2d()), () -> controller.getY(), () -> controller.getX()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
