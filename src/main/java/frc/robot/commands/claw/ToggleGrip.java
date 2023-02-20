// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ToggleGrip extends CommandBase {
  /** Creates a new ToggleGrip. */
private final Claw claw; 
  public ToggleGrip(Claw claw) {
   
    this.claw = claw;
    addRequirements(claw);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.toggleClaw();

  }

  // Called every time the scheduler runs while the command is scheduled.

  public boolean isFinished() {
    return true;
  }
}
