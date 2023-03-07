// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmToPose extends CommandBase {
  private final Arm arm;
  private final Supplier<Translation2d> poseSupplier;
  private Translation2d pose;
  public ArmToPose(Arm arm, Supplier<Translation2d> pose) {
    addRequirements(arm);
    this.arm = arm;
    this.poseSupplier = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pose = poseSupplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This won't do anything if the pose isn't reachable for the arm.
    arm.toPosition(pose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isAtDesiredPosition();
  }

  public static ArmToPose holdPose(Arm arm, Supplier<Translation2d> pose) {
    return new ArmToPose(arm, pose) {
      @Override
      public boolean isFinished() {
          return false;
      }
    };
  }

  public static ArmToPose holdPose(Arm arm) {
    return holdPose(arm, () -> arm.getPosition());
  }
}
