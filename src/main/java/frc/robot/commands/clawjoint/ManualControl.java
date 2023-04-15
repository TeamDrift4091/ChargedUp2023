// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.clawjoint;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawJoint;

public class ManualControl extends CommandBase {
  private final ClawJoint clawJoint;
  private final BooleanSupplier up, down;
  private double targetAngleRadians;
  public ManualControl(ClawJoint clawJoint, BooleanSupplier up, BooleanSupplier down) {
    addRequirements(clawJoint);
    this.clawJoint = clawJoint;
    this.up = up;
    this.down = down;

    targetAngleRadians = .1; // just a random number, just to be safe
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (up.getAsBoolean()) {
      // targetAngleRadians += .05;
      clawJoint.drive(.5);
    } else
    if (down.getAsBoolean()) {
      // targetAngleRadians -= .05;
      clawJoint.drive(-.3);
    } else {
      clawJoint.drive(0);
    }
    // targetAngleRadians = MathUtil.clamp(targetAngleRadians, ClawJointConstants.MIN_ANGLE, ClawJointConstants.MAX_ANGLE);
    // clawJoint.setAngle(targetAngleRadians);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
