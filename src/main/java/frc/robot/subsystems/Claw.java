package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private WPI_TalonFX motor1;
    private WPI_TalonFX motor2;

    public Claw(int motor1ID, int motor2ID) {
        motor1 = new WPI_TalonFX(motor1ID);
        motor2 = new WPI_TalonFX(motor2ID);

        motor1.configFactoryDefault();
        motor2.configFactoryDefault();
    }

    public void setMotorSpeed(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }
}