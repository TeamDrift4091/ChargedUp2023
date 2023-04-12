package frc.robot.commands.clawjoint;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.subsystems.ClawJoint;

public class ClawToAngle extends CommandBase {
  private final ClawJoint clawJoint;
  private final double targetAngleRadians;

  public ClawToAngle(ClawJoint clawJoint, double angleRadians) {
    addRequirements(clawJoint);
    this.clawJoint = clawJoint;
    this.targetAngleRadians = angleRadians;
  }

  public ClawToAngle(ClawJoint clawJoint, ScoringLevel scoringLevel) {
    this(clawJoint, scoringLevel.getRequiredClawAngle());
  }

  @Override
  public void initialize() {
    clawJoint.setAngle(targetAngleRadians);
  }
}