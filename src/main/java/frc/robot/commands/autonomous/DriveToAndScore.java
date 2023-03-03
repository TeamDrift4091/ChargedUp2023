// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmToPose;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.commands.drivetrain.DriveToPose;
import frc.robot.subsystems.*;
import frc.robot.utility.GameObject;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToAndScore extends SequentialCommandGroup {
  public DriveToAndScore(Drivetrain drivetrain, Arm arm, Claw claw, GameObject gameObject, ScoringLevel scoringLevel) {
    addCommands(
      new DriveToPose(drivetrain, () -> ScoringLocationManager.getNearestScoringPosition(gameObject)),
      new ArmToPose(arm, () -> ScoringLocationManager.getArmPosition(gameObject, scoringLevel)),
      new ParallelCommandGroup(
        // new ConditionalCommand(
        //   new DropFromClaw(claw),
        //   new EjectFromClaw(claw),
        //   () -> gameObject.equals(GameObject.CONE)
        // ),
        ArmToPose.holdPose(arm, () -> ScoringLocationManager.getArmPosition(gameObject, scoringLevel))
      ).withTimeout(1)
    );
  }

  public DriveToAndScore(Drivetrain drivetrain, Arm arm, Claw claw, Supplier<GameObject> gameObject, ScoringLevel scoringLevel) {
    addCommands(
      new DriveToPose(drivetrain, () -> ScoringLocationManager.getNearestScoringPosition(gameObject.get())),
      new ArmToPose(arm, () -> ScoringLocationManager.getArmPosition(gameObject.get(), scoringLevel)),
      new ParallelCommandGroup(
        // new ConditionalCommand(
        //   new DropFromClaw(claw),
        //   new EjectFromClaw(claw),
        //   () -> gameObject.equals(GameObject.CONE)
        // ),
        ArmToPose.holdPose(arm, () -> ScoringLocationManager.getArmPosition(gameObject.get(), scoringLevel))
      ).withTimeout(1)
    );
  }
}
