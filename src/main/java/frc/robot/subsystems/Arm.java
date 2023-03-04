package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.Subsystem;

public class Arm extends Subsystem {
    private final WPI_TalonFX climberMotor; //declares a variable "climberMotor" of type "TalonFX" which represents the climber motor.
    private final WPI_TalonFX clawString; //declares a variable "clawString" of type "TalonFX" which represents the claw string motor
    private final WPI_TalonFX shoulderMotor; // declares a variable "shoulderMotor" of type "WPI_TalonFX" which represents the motor of the shoulder joint

    // Sim / SmartDashboard visuals
    // See https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
    private final Mechanism2d armMechanism2d = new Mechanism2d(ArmConstants.ARM_MAX_LENGTH*1.5, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND*1.5);
    private final MechanismRoot2d armRoot = armMechanism2d.getRoot("shoulder", ArmConstants.SHOULDER_HEIGHT_FROM_GROUND*.25, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND);
    private final MechanismLigament2d arm = armRoot.append(new MechanismLigament2d("arm", ArmConstants.ARM_MIN_LENGTH, -90));
    private final MechanismLigament2d claw = arm.append(new MechanismLigament2d("claw", .1, 90));

    private final double startingAngleRadians = -Math.PI/2.;

    public Arm(){
        climberMotor = new WPI_TalonFX(ArmConstants.CLIMBER_ID); //creates a new talonFX instance for the climber motor using its CAN ID
        clawString = new WPI_TalonFX(ArmConstants.CLAW_STRING_ID); //creates a nnew TalonFx instance for the claw string motor using its CAN ID
        shoulderMotor = new WPI_TalonFX(ArmConstants.SHOULDER_ID); // creates a new WPI_TalonFX instance for the shoulder of the robot arm using its CAN ID

        configDriveMotor(climberMotor);
        configDriveMotor(clawString);
        configDriveMotor(shoulderMotor);
        // Assume the robot starts with its arm down and fully retracted.
        reset();
        // Configure SmartDashboard and arm visualization.
        configureSmartDashboard();
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
        climberMotor.setSelectedSensorPosition(0);
        clawString.setSelectedSensorPosition(0);
        shoulderMotor.setSelectedSensorPosition(startingAngleRadians / (2*Math.PI) * ArmConstants.SHOULDER_GEAR_RATIO * 2048);
    }

    public void toHome() {
        toExtension(0);
        toAngle(new Rotation2d(startingAngleRadians));
    }

    /**
     * Sets the motors controlling the arm extension.
     */
    public void setExtension(ControlMode controlMode, double value){  //method that takes in ControlMode and a double value as parameters, sets the control mode and value parallel for all three motors
        climberMotor.set(controlMode, value); //sets control mode and value for the climber motor
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
     * @param position the target position
     * @return whether or not the target position is valid
     */
    public boolean toPosition(Translation2d position) {
        Translation2d target = position.minus((new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND)));
        return toExtension(target.getNorm()) && toAngle(target.getAngle());
    }

    /**
     * Returns false if the extensions length is impossible for our arm.
     */
    private boolean toExtension(double extensionMeters) {
        if (ArmConstants.ARM_MIN_LENGTH > extensionMeters || extensionMeters > ArmConstants.ARM_MAX_LENGTH) {
            return false;
        }
        // Handle sim visualization
        if (Robot.isSimulation()) {
            arm.setLength(extensionMeters);
        }
        // Convert from meters to encoder ticks
        double target = (extensionMeters - ArmConstants.ARM_MIN_LENGTH) * ArmConstants.ARM_GEAR_RATIO / (2048*ArmConstants.METERS_PER_CLIMBER_ROTATION);
        setExtension(ControlMode.Position, target);
        
        return true;
    }

    /**
     * Returns false if the rotation is impossible for our arm.
     */
    private boolean toAngle(Rotation2d rotation) {
        if (rotation.getRadians() - startingAngleRadians > ArmConstants.ARM_MAX_ANGLE || rotation.getRadians() - startingAngleRadians < 0) {
            return false;
        }
        // Handle sim visualization
        if (Robot.isSimulation()) {
            arm.setAngle(rotation);
            claw.setAngle(-rotation.getDegrees());
        }
        // Add offset since a rotation is relative to the horizontal, and we want it relative to straight down.
        double target = (rotation.getRadians() * ArmConstants.SHOULDER_GEAR_RATIO / (2048 * 2 * Math.PI));
        setShoulder(ControlMode.Position, target);

        return true;
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
    public double getArmExtensionDistance() {
        // Handle sim visualization
        if (Robot.isSimulation()) {
            return arm.getLength();
        }
        // Calculates the extension of the arm based on the climber motor.
        // TODO: This may cause issues since we are just measuring from the left motor.
        return ArmConstants.ARM_MIN_LENGTH + (climberMotor.getSelectedSensorPosition() / ArmConstants.ARM_GEAR_RATIO * ArmConstants.METERS_PER_CLIMBER_ROTATION);
    }

    /**
     * Returns a rotation, representing the angle of the arm relative to a straight down position
     * @return the rotation of the arm
     */
    public Rotation2d getArmAngle() {
        // Handle sim visualization
        if (Robot.isSimulation()) {
            return Rotation2d.fromDegrees(arm.getAngle());
        }
        // Calculates the angle of the arm based on the encoder measurement of the shoulder motor.
        return new Rotation2d(shoulderMotor.getSelectedSensorPosition()/2048./ArmConstants.SHOULDER_GEAR_RATIO * 2 * Math.PI);
    }

    @Override
    protected void configureSmartDashboard() {
        SmartDashboard.putData("Arm (Mechanism2d)", armMechanism2d);

        LazyDashboard.addNumber("Arm/x", getPosition()::getX);
        LazyDashboard.addNumber("Arm/y", getPosition()::getY);
    }

    @Override
    public void stop() {
        climberMotor.stopMotor();
        clawString.stopMotor();
        shoulderMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // Update the visualization with real values.
        if (Robot.isReal()) {
            arm.setAngle(getArmAngle());
            claw.setAngle(-getArmAngle().getDegrees());
            arm.setLength(getArmExtensionDistance());
        }
    }
    
    private static void configDriveMotor(WPI_TalonFX driveMotor){
        driveMotor.configFactoryDefault(); // resets the motor to its factory default settings. 
        driveMotor.setNeutralMode(NeutralMode.Brake); //sets neutral mode of robot to break.
        driveMotor.config_kP(0, 1);
        driveMotor.config_kI(0, 0);
        driveMotor.config_kD(0, 0);
        driveMotor.config_kF(0, 0);

    }
}
