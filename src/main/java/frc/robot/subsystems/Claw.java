package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Claw extends SubsystemBase {
  private DoubleSolenoid gripSolenoid;
  private WPI_TalonFX leftClawMotor;
  private WPI_TalonFX rightClawMotor;

  public Claw() {
    gripSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    leftClawMotor = new WPI_TalonFX(0);
    rightClawMotor = new WPI_TalonFX(1);
  }

  public void setGrip(DoubleSolenoid.Value position) {
    gripSolenoid.set(position);
  }

  public void setClawSpeed(double speed) {
    leftClawMotor.set(speed);
    rightClawMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSolenoidPositions(Value kforward) {
  }

public void setMotors(double d) {
}
}