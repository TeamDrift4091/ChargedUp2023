package frc.robot.commands.clawjoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawJointConstants;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.subsystems.ClawJoint;

public class ClawToAngle extends CommandBase {
  private final ClawJoint clawJoint;
  private final double targetAngleRotations;

  public ClawToAngle(ClawJoint clawJoint, double angleRotations) {
    addRequirements(clawJoint);
    this.clawJoint = clawJoint;
    this.targetAngleRotations = angleRotations;
    
    SmartDashboard.putNumber("ClawJoint/targetAngle", targetAngleRotations);
  }

  public ClawToAngle(ClawJoint clawJoint, ScoringLevel scoringLevel) {
    this(clawJoint, scoringLevel.getRequiredClawAngle());
  }

  @Override
  public void execute() {
    clawJoint.setAngle(MathUtil.clamp(SmartDashboard.getNumber("ClawJoint/targetAngle", targetAngleRotations), ClawJointConstants.MIN_ANGLE, ClawJointConstants.MAX_ANGLE));
  }

  public static ClawToAngle intake(ClawJoint clawJoint) {
    return new ClawToAngle(clawJoint, 0);
  }
}