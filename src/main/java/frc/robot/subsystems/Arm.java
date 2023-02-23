package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    
    
private WPI_TalonFX leftClimber; //declares a variable "leftClimber" of type "TalonFX" which represents the left climber motor.
private WPI_TalonFX rightClimber; //declares a variable "rightClimber" of type "TalonFX" which represents the right climber motor
private WPI_TalonFX clawString; //declares a variable "clawString" of type "TalonFX" which represents the claw string motor

    public Arm(){
        leftClimber = new WPI_TalonFX(Constants.ArmConstants.LEFT_CLIMBER_ID); //creates a new talonFX instance for the left climber motor  using its CAN ID
        rightClimber = new WPI_TalonFX(Constants.ArmConstants.RIGHT_CLIMBER_ID); //creates a new TalonFx instance for the right climber motor using its CAN ID
        clawString = new WPI_TalonFX(Constants.ArmConstants.CLAW_STRING_ID); //creates a nnew TalonFx instance for the claw string motor using its CAN ID
    }
    public void set(ControlMode controlMode, double value){  //method that takes in ControlMode and a double value as parameters, sets the control mode and value parallel for all three motors
        leftClimber.set(controlMode, value); //sets control mode and value for the left climber motor
        rightClimber.set(controlMode, value); //sets control mode and value for the right climber motor
        clawString.set(controlMode, value); //sets control mode and value for the claw string motor
        
    
    }


}
