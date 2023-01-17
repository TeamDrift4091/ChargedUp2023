// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.subsystems.Drivetrain;
import frc.team1891.common.control.SmartController;

public class RobotContainer {

  // Subsystems
  Drivetrain drivetrain = Drivetrain.getInstance();

  // Controllers
  SmartController controller = new SmartController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
      new JoystickDrive(
        drivetrain,
        () -> controller.getForwardAxis(),
        () -> controller.getStrafeAxis(),
        () -> controller.getTwistAxis()
      )
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void updateSmartControllers() {
    System.out.println("Info: Updating SmartControllers.");
    controller.configure();
  }
}
