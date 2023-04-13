// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.commands.claw.ShootWithDelay;
import frc.robot.commands.clawjoint.ClawToAngle;
import frc.robot.commands.clawjoint.HomeClawPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawJoint;

public class ShootFull extends ParallelCommandGroup {
  public ShootFull(Claw claw, ClawJoint clawJoint, ScoringLevel scoringLevel, boolean shootBackwards) {
    if (shootBackwards) {
      addCommands(new HomeClawPosition(clawJoint));
    } else {
      addCommands(new ClawToAngle(clawJoint, scoringLevel));
    }
    addCommands(new ShootWithDelay(claw, scoringLevel));
  }
}
