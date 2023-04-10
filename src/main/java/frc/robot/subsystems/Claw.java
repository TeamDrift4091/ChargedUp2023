package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
    private final WPI_TalonFX innerLeftMotor, innerRightMotor, outerLeftMotor, outerRightMotor;

    public Claw() {
        innerLeftMotor = new WPI_TalonFX(ClawConstants.INNER_LEFT_MOTOR_ID);
        innerRightMotor = new WPI_TalonFX(ClawConstants.INNER_RIGHT_MOTOR_ID);
        outerLeftMotor = new WPI_TalonFX(ClawConstants.OUTER_LEFT_MOTOR_ID);
        outerRightMotor = new WPI_TalonFX(ClawConstants.OUTER_RIGHT_MOTOR_ID);

        innerLeftMotor.configFactoryDefault();
        innerRightMotor.configFactoryDefault();
        innerLeftMotor.setNeutralMode(NeutralMode.Brake);
        innerRightMotor.setNeutralMode(NeutralMode.Brake);

        outerLeftMotor.configFactoryDefault();
        outerRightMotor.configFactoryDefault();
        outerLeftMotor.setNeutralMode(NeutralMode.Coast);
        outerRightMotor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Sets the speed of the outer motors
     */
    public void setShootingMotors(double speed) {
        outerLeftMotor.set(speed);
        outerRightMotor.set(speed);
    }

    /**
     * Sets the speed of the inner motors
     */
    public void setInnerMotors(double speed) {
        innerLeftMotor.set(speed);
        innerRightMotor.set(speed);
    }

    /**
     * Sets the speed of all motors in the claw
     */
    public void setAllMotors(double speed) {
        setShootingMotors(speed);
        setInnerMotors(speed);
    }

    @Override
    public void periodic() {}
}