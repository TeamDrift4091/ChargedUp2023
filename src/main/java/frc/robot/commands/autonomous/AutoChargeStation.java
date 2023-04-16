// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.BalanceOnChargingStationLinear;
import frc.robot.commands.drivetrain.FaceModules;
import frc.robot.subsystems.Drivetrain;

public class AutoChargeStation extends SequentialCommandGroup {
  /** Creates a new AutoChargeStation. */
  public AutoChargeStation(Drivetrain drivetrain) {
    addCommands(
      // Reset the gyro before doing anything
      new InstantCommand(() -> {
        drivetrain.resetGyro();
        BalanceOnChargingStationLinear.calibrateOffset();
      }, drivetrain),

      // Align wheels forward before we start moving
      new FaceModules(drivetrain).withTimeout(1), // with a timeout just in case it doesn't end properly

      // Drive forward until the gyro angle is steep enough
      new RunCommand(() -> {
        drivetrain.fromChassisSpeeds(new ChassisSpeeds(1, 0, 0)); // drive forward at 1 m/s
      }, drivetrain) {
        @Override
        public void end(boolean interrupted) {
          drivetrain.stop();
        }
        @Override
        public boolean isFinished() {
            return Math.abs(drivetrain.getGyroMeasurement().getY()) > BalanceOnChargingStationLinear.balanceTolerance; // is pitch steeper than .2 rad
        }
      },

      // Attempt to balance
      new BalanceOnChargingStationLinear(drivetrain) // attempt to balance until gyro reads close to 0 in pitch
    );
  }
}
