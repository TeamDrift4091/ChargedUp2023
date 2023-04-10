// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class BalanceOnChargingStationLinear extends CommandBase {
  private final Drivetrain drivetrain;
  private final double kP = 2; // proportion coefficient
  
  public BalanceOnChargingStationLinear(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation3d gyroMeasurement = drivetrain.getGyroMeasurement();

    double pitch = gyroMeasurement.getY(); // Rotation around the y axis

    drivetrain.fromChassisSpeeds(
      new ChassisSpeeds(
        pitch * kP,
        0,
        0
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return drivetrain.getGyroMeasurement().getY() < .08; // Made up number
  }
}
