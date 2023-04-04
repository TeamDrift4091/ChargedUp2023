// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.commands.drivetrain.DriveToPose;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToAndScore extends SequentialCommandGroup {
  public DriveToAndScore(Drivetrain drivetrain, ScoringLevel scoringLevel) {
    // If the target is to score low, we can align to any node, otherwise we need to go specifically to a cube node.
    if (scoringLevel == ScoringLevel.HYBRID) {
      addCommands(new DriveToPose(drivetrain, () -> ScoringLocationManager.getNearestNodeAlignment()));
    } else {
      addCommands(new DriveToPose(drivetrain, () -> ScoringLocationManager.getNearestCubeNodeAlignment()));
    }

    // addCommands(new Shoot(scoringLevel));
  }
}
