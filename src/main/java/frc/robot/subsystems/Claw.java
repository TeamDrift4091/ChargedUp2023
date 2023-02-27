package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Claw extends SubsystemBase {
  private DoubleSolenoid gripSolenoid;
  private WPI_TalonFX leftClawMotor;
  private WPI_TalonFX rightClawMotor;

  public Claw() {
    gripSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConstants.PNEUMATIC_FORWORD_CHANNEL, ClawConstants.PNEUMATIC_BACK_CHANNEL);
    leftClawMotor = new WPI_TalonFX(ClawConstants.LEFT_CLAW_CHANNEL);
    rightClawMotor = new WPI_TalonFX(ClawConstants.RIGHT_CLAW_CHANNEL);
    configDriveMotor(leftClawMotor);
    configDriveMotor(rightClawMotor);

  }

  /**
   * Opens the grip solenoid
   */
  public void openGrip() {
    gripSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Closes the grip solenoid
   */
  public void closeGrip() {
    gripSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  
  /**
   * Sets the speed of the wheels in the claw
   * @param speed
   */
  public void setClawSpeed(double speed) {
    leftClawMotor.set(speed);
    rightClawMotor.set(speed);
  }

  /**
   * Toggles the claw open or closed.
   */
  public void toggleClaw() {
    gripSolenoid.get();
      if (gripSolenoid.get().equals(DoubleSolenoid.Value.kReverse)) {
          openGrip();// Open the claw
      } else {
        closeGrip();// Close the claw
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  private static void configDriveMotor(WPI_TalonFX driveMotor){
    driveMotor.configFactoryDefault(); // resets the motor to its factory default settings. 
    driveMotor.setNeutralMode(NeutralMode.Brake); //sets neutral mode of robot to break.
    driveMotor.config_KP(1, p);
    driveMotor.config_KI(0, i);
    driveMotor.config_KD(0, d);
    driveMotor.config_KF(0, f);

  }
}