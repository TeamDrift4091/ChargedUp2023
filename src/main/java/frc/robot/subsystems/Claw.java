package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Claw extends SubsystemBase {
  private DoubleSolenoid gripSolenoid;
  private WPI_TalonFX leftClawMotor;
  private WPI_TalonFX rightClawMotor;

  public Claw() {
    gripSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Claw.PNEUMATIC_FORWORD_CHANNEL, Constants.Claw.PNEUMATIC_BACK_CHANNEL);
    leftClawMotor = new WPI_TalonFX(Constants.Claw.LEFT_CLAW_CHANNEL);
    rightClawMotor = new WPI_TalonFX(Constants.Claw.RIGHT_CLAW_CHANNEL);
  }
  public void openGrip() {
    gripSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void closeGrip() {
    gripSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void setClawSpeed(double speed) {
    leftClawMotor.set(speed);
    rightClawMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}