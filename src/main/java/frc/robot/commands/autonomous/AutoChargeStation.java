// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.BalanceOnChargingStation;
import frc.robot.subsystems.Drivetrain;

public class AutoChargeStation extends SequentialCommandGroup {
  /** Creates a new AutoChargeStation. */
  public AutoChargeStation(Drivetrain drivetrain) {
    addCommands(
      // Reset the gyro before doing anything
      new InstantCommand(() -> {
        drivetrain.resetGyro();
      }, drivetrain),

      // Drive forward until the gyro angle is steep enough
      new RunCommand(() -> {
        drivetrain.fromChassisSpeeds(new ChassisSpeeds(1, 0, 0)); // drive forward at 1 m/s
      }, drivetrain) {
        @Override
        public boolean isFinished() {
            return Math.abs(drivetrain.getGyroMeasurement().getY()) > .2; // is pitch steeper than .3 rad
        }
      },

      // Attempt to balance
      new BalanceOnChargingStation(drivetrain) // drive in the 'steepest' direction, and stop if it's flat
    );
  }
}
