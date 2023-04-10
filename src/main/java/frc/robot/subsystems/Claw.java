package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private WPI_TalonFX motor1;
    private WPI_TalonFX motor2;
    private WPI_TalonFX motor3;
    private WPI_TalonFX motor4;

    public Claw(int motor1ID, int motor2ID, int motor3ID, int motor4ID) {
        motor1 = new WPI_TalonFX(motor1ID);
        motor2 = new WPI_TalonFX(motor2ID);
        motor3 = new WPI_TalonFX(motor3ID);
        motor4 = new WPI_TalonFX(motor4ID);



        motor1.configFactoryDefault();
        motor2.configFactoryDefault();
        motor3.configFactoryDefault();
        motor4.configFactoryDefault();
    }

    public void setMotorSpeed(double speed) {
        motor1.set(speed);
        motor2.set(speed);
        motor3.set(speed);
        motor4.set(speed);
    }
}