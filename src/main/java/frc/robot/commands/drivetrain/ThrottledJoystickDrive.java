// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Drivetrain;

public class ThrottledJoystickDrive extends JoystickDrive {
  private final DoubleSupplier throttle;
  /**
   * Creates a new command that drives the robot according to joystick inputs (Double Suppliers). This is field oriented.
   * @param drivetrain
   * @param forward
   * @param strafe
   * @param twist
   */
  public ThrottledJoystickDrive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier twist, DoubleSupplier throttle) {
    super(drivetrain, forward, strafe, twist);
    this.throttle = throttle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttlePercent = MathUtil.clamp(throttle.getAsDouble(), 0, 1);
    drivetrain.holonomicDrive(-forward.getAsDouble()*throttlePercent, -strafe.getAsDouble()*throttlePercent, -twist.getAsDouble()*throttlePercent, true); // negative is forward on the joystick; chassis left is positive while joystick right is positive.
  }
}
