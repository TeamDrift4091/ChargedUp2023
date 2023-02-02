// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends CommandBase {
  public static final double DEADBAND = .15;

  private static final boolean SMARTDASHBOARD = false;

  private final Drivetrain drivetrain;
  private final DoubleSupplier forward, strafe, twist;
  public JoystickDrive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier twist) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.twist = twist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double f = MathUtil.applyDeadband(forward.getAsDouble(), DEADBAND);
    double s = MathUtil.applyDeadband(strafe.getAsDouble(), DEADBAND);
    double t = MathUtil.applyDeadband(twist.getAsDouble(), DEADBAND);
    drivetrain.holonomicDrive(-f, -s, t, true); // negative is forward on the joystick; chassis left is positive while joystick right is positive.

    if (SMARTDASHBOARD) {
      SmartDashboard.putNumber("joystick/forward", forward.getAsDouble());
      SmartDashboard.putNumber("joystick/strafe", strafe.getAsDouble());
      SmartDashboard.putNumber("joystick/twist", twist.getAsDouble());
      SmartDashboard.putNumber("joystick/forward deadband", f);
      SmartDashboard.putNumber("joystick/strafe deadband", s);
      SmartDashboard.putNumber("joystick/twist deadband", t);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
