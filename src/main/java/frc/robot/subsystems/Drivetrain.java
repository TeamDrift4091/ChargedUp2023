// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.team1891.common.drivetrains.DrivetrainConfig;
import frc.team1891.common.drivetrains.SwerveDrivetrain;
import frc.team1891.common.drivetrains.SwerveModule;
import frc.team1891.common.hardware.NavX;

public class Drivetrain extends SwerveDrivetrain {
  private static Drivetrain instance;
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  private static final ShuffleboardTab _shuffuleboardTab = Shuffleboard.getTab("Drivetrain");

  private static final DrivetrainConfig _config = new DrivetrainConfig(2, 1, Math.PI, Math.PI, 2, Constants.Drivetrain.DRIVETRAIN_GEAR_RATIO, 2048);

  private static final NavX _gyro = new NavX();

  // TODO: Tune PID
  private static final WPI_TalonFX frontLeftDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.FRONT_LEFT_DRIVE_CHANNEL);
  private static final WPI_TalonFX frontLeftSteerFalcon = new WPI_TalonFX(Constants.Drivetrain.FRONT_LEFT_STEER_CHANNEL);
  private static final WPI_CANCoder frontLeftEncoder = new WPI_CANCoder(Constants.Drivetrain.FRONT_LEFT_CANCODER_CHANNEL);
  private static final SwerveModule frontLeft = SwerveModule.createFromDriveFalconAndSteerFalcon(frontLeftDriveFalcon, frontLeftSteerFalcon, frontLeftEncoder, _config, 1,0,0,1,0,0);
  private static final WPI_TalonFX frontRightDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.FRONT_RIGHT_DRIVE_CHANNEL);
  private static final WPI_TalonFX frontRightSteerFalcon = new WPI_TalonFX(Constants.Drivetrain.FRONT_RIGHT_STEER_CHANNEL);
  private static final WPI_CANCoder frontRightEncoder = new WPI_CANCoder(Constants.Drivetrain.FRONT_RIGHT_CANCODER_CHANNEL);
  private static final SwerveModule frontRight = SwerveModule.createFromDriveFalconAndSteerFalcon(frontRightDriveFalcon, frontRightSteerFalcon, frontRightEncoder, _config, 1,0,0,1,0,0);
  private static final WPI_TalonFX backLeftDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.BACK_LEFT_DRIVE_CHANNEL);
  private static final WPI_TalonFX backLeftSteerFalcon = new WPI_TalonFX(Constants.Drivetrain.BACK_LEFT_STEER_CHANNEL);
  private static final WPI_CANCoder backLeftEncoder = new WPI_CANCoder(Constants.Drivetrain.BACK_LEFT_CANCODER_CHANNEL);
  private static final SwerveModule backLeft = SwerveModule.createFromDriveFalconAndSteerFalcon(backLeftDriveFalcon, backLeftSteerFalcon, backLeftEncoder, _config, 1,0,0,1,0,0);
  private static final WPI_TalonFX backRightDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.BACK_RIGHT_DRIVE_CHANNEL);
  private static final WPI_TalonFX backRightSteerFalcon = new WPI_TalonFX(Constants.Drivetrain.BACK_RIGHT_STEER_CHANNEL);
  private static final WPI_CANCoder backRightEncoder = new WPI_CANCoder(Constants.Drivetrain.BACK_RIGHT_CANCODER_CHANNEL);
  private static final SwerveModule backRight = SwerveModule.createFromDriveFalconAndSteerFalcon(backRightDriveFalcon, backRightSteerFalcon, backRightEncoder, _config, 1,0,0,1,0,0);

  private Drivetrain() {
    super(
      _shuffuleboardTab,
      _config, 
      Constants.Drivetrain.WHEEL_BASE_WIDTH_METERS,
      Constants.Drivetrain.WHEEL_BASE_LENGTH_METERS,
      _gyro,
      frontLeft,
      frontRight,
      backLeft,
      backRight
    );

    configDriveMotor(frontLeftDriveFalcon);
    configDriveMotor(frontRightDriveFalcon);
    configDriveMotor(backLeftDriveFalcon);
    configDriveMotor(backRightDriveFalcon);
    configSteerMotor(frontLeftSteerFalcon);
    configSteerMotor(frontRightSteerFalcon);
    configSteerMotor(backLeftSteerFalcon);
    configSteerMotor(backRightSteerFalcon);
    configCANCoder(frontLeftEncoder, Constants.Drivetrain.FRONT_LEFT_ENCODER_OFFSET);
    configCANCoder(frontRightEncoder, Constants.Drivetrain.FRONT_RIGHT_ENCODER_OFFSET);
    configCANCoder(backLeftEncoder, Constants.Drivetrain.BACK_LEFT_ENCODER_OFFSET);
    configCANCoder(backRightEncoder, Constants.Drivetrain.BACK_RIGHT_ENCODER_OFFSET);


    configureShuffleboard();
  }

  private static void configDriveMotor(WPI_TalonFX driveMotor) {
    driveMotor.configFactoryDefault();
    driveMotor.setNeutralMode(NeutralMode.Brake);
  }

  private static void configSteerMotor(WPI_TalonFX steerMotor) {
    steerMotor.configFactoryDefault();
    steerMotor.setNeutralMode(NeutralMode.Brake);
  }

  private static void configCANCoder(WPI_CANCoder encoder, double encoderOffset) {
    encoder.configFactoryDefault();
    encoder.configMagnetOffset(encoderOffset);
  }
}
