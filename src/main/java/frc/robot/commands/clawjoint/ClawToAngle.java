package frc.robot.commands.clawjoint;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public void execute() {
    SmartDashboard.putNumber("ClawJoint/targetAngle", targetAngleRadians);
    clawJoint.setAngle(targetAngleRadians);
  }

  public static ClawToAngle intake(ClawJoint clawJoint) {
    return new ClawToAngle(clawJoint, 0);
  }
}