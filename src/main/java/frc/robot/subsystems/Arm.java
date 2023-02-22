package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Arm {
    public final class Constants{
        public static final int LEFT_CLIMBER_ID = 1; 
        public static final int RIGHT_CLIMBER_ID = 2;
        public static final int CLAW_STRING_ID = 3;
        //CAN IDs for three motors

        public static final double JOINT_HEIGHT_FROM_GROUND =1;
        //joint height from ground 

        public static final double ARM_MIN_LENGTH = 0.5;
        public static final double ARM_MAX_LENGTH = 1;
        //Arm length limits
    }
    public class ClimberSubsystem{
        private TalonFX leftClimber; //declares a variable "leftClimber" of type "TalonFX" which represents the left climber motor.
        private TalonFX rightClimber; //declares a variable "rightClimber" of type "TalonFX" which represents the right climber motor
        private TalonFX clawString; //declares a variable "clawString" of type "TalonFX" which represents the claw string motor

        public ClimberSubsystem(){
            leftClimber = new TalonFX(1); //creates a new talonFX instance for the left climber motor  using its CAN ID
            rightClimber = new TalonFX(2); //creates a new TalonFx instance for the right climber motor using its CAN ID
            clawString = new TalonFX(3); //creates a nnew TalonFx instance for the claw string motor using its CAN ID
        }
        public void set(ControlMode controlMode, double value){  //method that takes in ControlMode and a double value as parameters, sets the control mode and value parallel for all three motors
            leftClimber.set(controlMode, value); //sets control mode and value for the left climber motor
            rightClimber.set(controlMode, value); //sets control mode and value for the right climber motor
            clawString.set(controlMode, value); //sets control mode and value for the claw string motor
        }

    
    }


}
