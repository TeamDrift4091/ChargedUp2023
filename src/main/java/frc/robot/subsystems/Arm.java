package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;;

public class Arm extends SubsystemBase {    
    private final WPI_TalonFX leftClimber; //declares a variable "leftClimber" of type "TalonFX" which represents the left climber motor.
    private final WPI_TalonFX rightClimber; //declares a variable "rightClimber" of type "TalonFX" which represents the right climber motor
    private final WPI_TalonFX clawString; //declares a variable "clawString" of type "TalonFX" which represents the claw string motor
    private final WPI_TalonFX shoulderMotor; // declares a variable "shoulderMotor" of type "WPI_TalonFX" which represents the motor of the shoulder joint

    public Arm(){
        leftClimber = new WPI_TalonFX(ArmConstants.LEFT_CLIMBER_ID); //creates a new talonFX instance for the left climber motor  using its CAN ID
        rightClimber = new WPI_TalonFX(ArmConstants.RIGHT_CLIMBER_ID); //creates a new TalonFx instance for the right climber motor using its CAN ID
        clawString = new WPI_TalonFX(ArmConstants.CLAW_STRING_ID); //creates a nnew TalonFx instance for the claw string motor using its CAN ID
        shoulderMotor = new WPI_TalonFX(ArmConstants.SHOULDER_ID); // creates a new WPI_TalonFX instance for the shoulder of the robot arm using its CAN ID

        // Assume the robot starts with its arm down and fully retracted.
        reset();
    }

    /** 
     * If for some reason the claw isn't level when the robot turns on, or it drifts as the match goes on, this method can be used to adjust the sensor measurement of the claw string
     */ 
    public void offsetClawString(double offset) {
        clawString.setSelectedSensorPosition(clawString.getSelectedSensorPosition() + offset);
    }

    /**
     * Resets motor encoders, assuming the robot arm is facing straight down and fully retracted, with a claw parallel to the ground.
     */
    public void reset() {
        leftClimber.setSelectedSensorPosition(0);
        rightClimber.setSelectedSensorPosition(0);
        clawString.setSelectedSensorPosition(0);
        shoulderMotor.setSelectedSensorPosition(0);
    }

    /**
     * Sets the motors controlling the arm extension.
     */
    public void setExtension(ControlMode controlMode, double value){  //method that takes in ControlMode and a double value as parameters, sets the control mode and value parallel for all three motors
        leftClimber.set(controlMode, value); //sets control mode and value for the left climber motor
        rightClimber.set(controlMode, value); //sets control mode and value for the right climber motor
        clawString.set(controlMode, value); //sets control mode and value for the claw string motor
    }

    /**
     * Sets the motor controlling the shoulder movement.
     */
    public void setShoulder(ControlMode controlMode, double value) {
        shoulderMotor.set(controlMode, value);
    }

    /**
     * Drive the robot arm to get the tip of the arm to the desired position relative to the robot center and the height above the ground.
     * @param position
     */
    public void toPosition(Translation2d position) {
        // TODO: If the arm wants to extend too far, don't let it.
        Translation2d target = position.minus((new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND)));
        toExtension(target.getNorm());
        toAngle(target.getAngle());
    }

    private void toExtension(double extensionMeters) {
        // Convert from meters to encoder ticks
        double target = (extensionMeters - ArmConstants.ARM_MIN_LENGTH) / (ArmConstants.ARM_GEAR_RATIO*ArmConstants.METERS_PER_CLIMBER_ROTATION);
        setExtension(ControlMode.Position, target);
    }

    private void toAngle(Rotation2d rotation) {
        // Add offset since a rotation is relative to the horizontal, and we want it relative to straight down.
        double target = (rotation.plus(Rotation2d.fromDegrees(90)).getRadians() / (ArmConstants.SHOULDER_GEAR_RATIO * 2 * Math.PI));
        setShoulder(ControlMode.Position, target);
    }

    /**
     * Returns the position of the end of the tip of the robot arm relative to the center of the robot, on the ground.
     * @return the position of the endeffector
     */
    public Translation2d getPosition() {
        return new Translation2d(getArmExtensionDistance(), getArmAngle()).plus(new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND));
    }

    /**
     * Returns a double, representing the length of the arm.
     * @return
     */
    private double getArmExtensionDistance() {
        // Calculates the extension of the arm based on the left climber motor.
        // TODO: This may cause issues since we are just measuring from the left motor.
        return ArmConstants.ARM_MIN_LENGTH + (leftClimber.getSelectedSensorPosition() * ArmConstants.ARM_GEAR_RATIO * ArmConstants.METERS_PER_CLIMBER_ROTATION);
    }

    /**
     * Returns a rotation, representing the angle of the arm relative to a straight down position
     * @return the rotation of the arm
     */
    private Rotation2d getArmAngle() {
        // Calculates the angle of the arm based on the encoder measurement of the shoulder motor.
        return new Rotation2d(shoulderMotor.getSelectedSensorPosition()*ArmConstants.SHOULDER_GEAR_RATIO * 2 * Math.PI);
    }
}
