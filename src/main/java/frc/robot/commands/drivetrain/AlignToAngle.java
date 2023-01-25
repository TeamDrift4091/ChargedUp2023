// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AlignToAngle extends CommandBase {
  private final Drivetrain drivetrain;
  private double angleDegrees;
  private final DoubleSupplier angleDegreesSupplier;
  private final ProfiledPIDController angleController;

  public AlignToAngle(Drivetrain drivetrain, double angleDegrees) {
    this.angleDegrees = angleDegrees;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.angleDegreesSupplier = null;

    angleController = new ProfiledPIDController(.007, .001, 0,
      new TrapezoidProfile.Constraints(
        drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond,
        drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared
      )
    );
    angleController.enableContinuousInput(0, 360);
  }

  public AlignToAngle(Drivetrain drivetrain, DoubleSupplier angleDegrees) {
    this.angleDegreesSupplier = angleDegrees;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.angleDegrees = Double.NaN;

    angleController = new ProfiledPIDController(.007, .001, 0,
      new TrapezoidProfile.Constraints(
        drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond,
        drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared
      )
    );
    angleController.enableContinuousInput(0, 360);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentAngle = drivetrain.getPose2d().getRotation().getDegrees();
    currentAngle %= 360.;
    if (currentAngle < 0) {currentAngle += 360;}
    angleController.reset(currentAngle);

    if (angleDegreesSupplier != null) {
      angleController.setGoal(angleDegreesSupplier.getAsDouble());
      angleDegrees = angleDegreesSupplier.getAsDouble();
    } else {
      angleController.setGoal(angleDegrees);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = drivetrain.getPose2d().getRotation().getDegrees();
    currentAngle %= 360.;
    if (currentAngle < 0) {currentAngle += 360;}
    SmartDashboard.putNumber("Current Angle", currentAngle);
    SmartDashboard.putNumber("Target Angle", angleDegrees);
    double output = angleController.calculate(currentAngle);

    double clampValue = .8;
    output = MathUtil.clamp(output, -clampValue, clampValue);
    drivetrain.holonomicDrive(0, 0, -output, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return angleController.atGoal();
    return false;
  }

  public static AlignToAngle alignToNearestSquare(Drivetrain drivetrain) {
    return new AlignToAngle(drivetrain, () -> {
      double currentAngle = drivetrain.getPose2d().getRotation().getDegrees();
      double targetAngle = Math.floor((currentAngle + 45) / 90.) * 90;
      return targetAngle;
    });
  }

  public static AlignToAngle alignForward(Drivetrain drivetrain) {
    return new AlignToAngle(drivetrain, 0);
  }
}
