// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawJointConstants;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.hardware.WPI_CANSparkMax;

public class ClawJoint extends SubsystemBase {
  // Singleton structure
  private static ClawJoint instance = null;
  public static ClawJoint getInstance() {
    if (instance == null) {
      instance = new ClawJoint();
    }
    return instance;
  }

  private final WPI_CANSparkMax motor;
  private final SparkMaxAbsoluteEncoder encoder;
  private final SparkMaxPIDController pidController;

  private ClawJoint() {
    motor = new WPI_CANSparkMax(ClawJointConstants.MOTOR_ID, WPI_CANSparkMax.MotorType.kBrushed);
    
    // Configure the motor
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    motor.setIdleMode(WPI_CANSparkMax.IdleMode.kBrake);

    encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    encoder.setInverted(true);
    pidController = motor.getPIDController();

    // Give the PID controller access to the encoder position
    pidController.setFeedbackDevice(encoder);

    // Configure PID
    pidController.setP(0);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setFF(.2);
    pidController.setOutputRange(-1, 1);

    // Sets the conversion factors, so when we ask for the encoder position it returns a value in radians
    encoder.setPositionConversionFactor((2 * Math.PI));
    encoder.setVelocityConversionFactor((2 * Math.PI) / 60.);

    // Give a SmartDashboard output of the angle
    LazyDashboard.addNumber("ClawJoint/angleRadians", this::getAngleRadians); // LazyDashboard updates the SmartDashboard value periodically using the getAngleRadians() method
  }

  /**
   * Drives the claw joint to the target angle
   * @param radians target angle
   */
  public void setAngle(double radians) {
    // Make sure the target angle is attainable before trying to move.
    if (ClawJointConstants.MIN_ANGLE <= radians && radians <= ClawJointConstants.MAX_ANGLE) {
      pidController.setReference(radians - ClawJointConstants.ENCODER_OFFSET_RADIANS, WPI_CANSparkMax.ControlType.kPosition);
    }
  }

  /**
   * Returns the current angle of the claw joint in radians
   * @return current angle in radians
   */
  public double getAngleRadians() {
    return encoder.getPosition() + ClawJointConstants.ENCODER_OFFSET_RADIANS;
  }

  @Override
  public void periodic() {}
}
