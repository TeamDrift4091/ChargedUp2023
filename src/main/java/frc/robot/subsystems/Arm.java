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
    private final WPI_TalonFX leftClimber; //declares a variable "leftClimber" of type "TalonFX" which represents the left climber motor.
    private final WPI_TalonFX rightClimber; //declares a variable "rightClimber" of type "TalonFX" which represents the right climber motor
    private final WPI_TalonFX clawString; //declares a variable "clawString" of type "TalonFX" which represents the claw string motor
    private final WPI_TalonFX shoulderMotor; // declares a variable "shoulderMotor" of type "WPI_TalonFX" which represents the motor of the shoulder joint

    // Sim / SmartDashboard visuals
    // See https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/mech2d-widget.html
    private final Mechanism2d armMechanism2d = new Mechanism2d(ArmConstants.ARM_MAX_LENGTH, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND*1.5);
    private final MechanismRoot2d armRoot = armMechanism2d.getRoot("shoulder", ArmConstants.SHOULDER_HEIGHT_FROM_GROUND*.25, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND);
    private final MechanismLigament2d arm = armRoot.append(new MechanismLigament2d("arm", ArmConstants.ARM_MIN_LENGTH, -90));
    private final MechanismLigament2d claw = arm.append(new MechanismLigament2d("claw", .1, 90));

    public Arm(){
        leftClimber = new WPI_TalonFX(ArmConstants.LEFT_CLIMBER_ID); //creates a new talonFX instance for the left climber motor  using its CAN ID
        rightClimber = new WPI_TalonFX(ArmConstants.RIGHT_CLIMBER_ID); //creates a new TalonFx instance for the right climber motor using its CAN ID
        clawString = new WPI_TalonFX(ArmConstants.CLAW_STRING_ID); //creates a nnew TalonFx instance for the claw string motor using its CAN ID
        shoulderMotor = new WPI_TalonFX(ArmConstants.SHOULDER_ID); // creates a new WPI_TalonFX instance for the shoulder of the robot arm using its CAN ID

        configDriveMotor(leftClimber);
        configDriveMotor(rightClimber);
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
        leftClimber.setSelectedSensorPosition(0);
        rightClimber.setSelectedSensorPosition(0);
        clawString.setSelectedSensorPosition(0);
        double startingAngleDegrees = -90;
        shoulderMotor.setSelectedSensorPosition(startingAngleDegrees / 360. * ArmConstants.SHOULDER_GEAR_RATIO * 2048);
        System.out.println("angleDegrees = "+(startingAngleDegrees / 360. * ArmConstants.SHOULDER_GEAR_RATIO * 2048));
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
        // Handle sim visualization
        if (Robot.isSimulation()) {
            arm.setLength(extensionMeters);
        }
        // Convert from meters to encoder ticks
        double target = (extensionMeters - ArmConstants.ARM_MIN_LENGTH) * ArmConstants.ARM_GEAR_RATIO / (2048*ArmConstants.METERS_PER_CLIMBER_ROTATION);
        setExtension(ControlMode.Position, target);
    }

    private void toAngle(Rotation2d rotation) {
        // Handle sim visualization
        if (Robot.isSimulation()) {
            arm.setAngle(rotation);
            claw.setAngle(-rotation.getDegrees());
        }
        // Add offset since a rotation is relative to the horizontal, and we want it relative to straight down.
        double target = (rotation.getRadians() * ArmConstants.SHOULDER_GEAR_RATIO / (2048 * 2 * Math.PI));
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
    public double getArmExtensionDistance() {
        // Handle sim visualization
        if (Robot.isSimulation()) {
            return arm.getLength();
        }
        // Calculates the extension of the arm based on the left climber motor.
        // TODO: This may cause issues since we are just measuring from the left motor.
        return ArmConstants.ARM_MIN_LENGTH + (leftClimber.getSelectedSensorPosition() / ArmConstants.ARM_GEAR_RATIO * ArmConstants.METERS_PER_CLIMBER_ROTATION);
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
        SmartDashboard.putData("Arm", armMechanism2d);

        LazyDashboard.addNumber("Arm/x", getPosition()::getX);
        LazyDashboard.addNumber("Arm/y", getPosition()::getY);
    }

    @Override
    public void stop() {
        leftClimber.stopMotor();
        rightClimber.stopMotor();
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
        driveMotor.config_KP(1, p);
        driveMotor.config_KI(0, i);
        driveMotor.config_KD(0, d);
        driveMotor.config_KF(0, f);

    }
}
