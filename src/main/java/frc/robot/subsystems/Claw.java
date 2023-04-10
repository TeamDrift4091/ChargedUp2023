package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private final WPI_TalonFX leftMotor, rightMotor;

    public Claw() {
        leftMotor = new WPI_TalonFX(ClawConstants.LEFT_MOTOR_ID);
        rightMotor = new WPI_TalonFX(ClawConstants.RIGHT_MOTOR_ID);

        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();
    }

    public void setMotorSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
}