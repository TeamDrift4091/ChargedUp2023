// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainTest extends CommandBase {
  private final Drivetrain drivetrain;
  public DrivetrainTest(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  private int count = 0;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (count < 360) {
      drivetrain.setSwerveModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(count)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(count)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(count)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(count))
        }
      );
    } else {
      drivetrain.setSwerveModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(.1, new Rotation2d()),
          new SwerveModuleState(.1, new Rotation2d()),
          new SwerveModuleState(.1, new Rotation2d()),
          new SwerveModuleState(.1, new Rotation2d())
        }
      );
    }
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count > 500;
  }
}
