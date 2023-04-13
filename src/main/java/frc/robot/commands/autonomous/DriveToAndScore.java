// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ShootFull;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.commands.drivetrain.DriveToPose;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToAndScore extends SequentialCommandGroup {
  public DriveToAndScore(Drivetrain drivetrain, Claw claw, ClawJoint clawJoint, ScoringLevel scoringLevel) {
    // If the target is to score low, we can align to any node, otherwise we need to go specifically to a cube node.
    if (scoringLevel == ScoringLevel.HYBRID) {
      addCommands(new DriveToPose(drivetrain, () -> ScoringLocationManager.getNearestNodeAlignment(), () -> rotate180(drivetrain)));
    } else {
      addCommands(new DriveToPose(drivetrain, () -> ScoringLocationManager.getNearestCubeNodeAlignment(), () -> rotate180(drivetrain)));
    }

    addCommands(new ConditionalCommand(
      new ShootFull(claw, clawJoint, scoringLevel, true),
      new ShootFull(claw, clawJoint, scoringLevel, false),
      () -> rotate180(drivetrain)));

    // addCommands(new Shoot(scoringLevel));
  }

  private static boolean rotate180(Drivetrain drivetrain) {
    double a = (drivetrain.getPose2d().getRotation().getDegrees() + 90) % 360.;
    a = a > 0 ? a : a + 360;
    boolean bool = a < 180;
    bool = Robot.isBlueAlliance() ? bool : !bool;
    return bool;
  }
}
