package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private DoubleSolenoid gripSolenoid;

  private static Claw instance;
  public static Claw getInstance() {
    if (instance == null) {
      instance = new Claw();
    }
    return instance;
  }

  private Claw() {
    gripSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConstants.PNEUMATIC_FORWORD_CHANNEL, ClawConstants.PNEUMATIC_BACK_CHANNEL);
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
   * Toggles the claw open or closed.
   */
  public void toggleGrip() {
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
}