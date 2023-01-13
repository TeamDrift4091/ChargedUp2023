// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  private static final DrivetrainConfig _config = new DrivetrainConfig(2, 1, Math.PI, Math.PI, 4, Constants.Drivetrain.DRIVETRAIN_GEAR_RATIO, 2048);

  private static double w = Constants.Drivetrain.WHEEL_BASE_WIDTH_METERS;
  private static double l = Constants.Drivetrain.WHEEL_BASE_LENGTH_METERS;
  private static double center = Constants.Drivetrain.WHEEL_BASE_CENTER_POINT_METERS;
  private static final Translation2d frontLeftLocation = new Translation2d(l-center, w/2.);
  private static final Translation2d frontRightLocation = new Translation2d(l-center, -w/2.);
  private static final Translation2d backLeftLocation = new Translation2d(-center, w/2.);
  private static final Translation2d backRightLocation = new Translation2d(-center, -w/2.);
  private static final SwerveDriveKinematics _kinematics = new SwerveDriveKinematics(
    frontLeftLocation,
    frontRightLocation,
    backLeftLocation,
    backRightLocation
  );

  private static final NavX _gyro = new NavX();

  // TODO: Tune PID
  private static final WPI_TalonFX frontLeftDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.FRONT_LEFT_DRIVE_CHANNEL);
  private static final WPI_TalonFX frontLeftSteerFalcon = new WPI_TalonFX(Constants.Drivetrain.FRONT_LEFT_STEER_CHANNEL);
  private static final CANCoder frontLeftEncoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_CANCODER_CHANNEL);
  private static final SwerveModule frontLeft = SwerveModule.createFromDriveFalconAndSteerFalcon(frontLeftDriveFalcon, frontLeftSteerFalcon, frontLeftEncoder, _config, 1,0,0,1,0,0);
  private static final WPI_TalonFX frontRightDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.FRONT_RIGHT_DRIVE_CHANNEL);
  private static final WPI_TalonFX frontRightSteerFalcon = new WPI_TalonFX(Constants.Drivetrain.FRONT_RIGHT_STEER_CHANNEL);
  private static final CANCoder frontRightEncoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_CANCODER_CHANNEL);
  private static final SwerveModule frontRight = SwerveModule.createFromDriveFalconAndSteerFalcon(frontRightDriveFalcon, frontRightSteerFalcon, frontRightEncoder, _config, 1,0,0,1,0,0);
  private static final WPI_TalonFX backLeftDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.BACK_LEFT_DRIVE_CHANNEL);
  private static final WPI_TalonFX backLeftSteerFalcon = new WPI_TalonFX(Constants.Drivetrain.BACK_LEFT_STEER_CHANNEL);
  private static final CANCoder backLeftEncoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_CANCODER_CHANNEL);
  private static final SwerveModule backLeft = SwerveModule.createFromDriveFalconAndSteerFalcon(backLeftDriveFalcon, backLeftSteerFalcon, backLeftEncoder, _config, 1,0,0,1,0,0);
  private static final WPI_TalonFX backRightDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.BACK_RIGHT_DRIVE_CHANNEL);
  private static final WPI_TalonFX backRightSteerFalcon = new WPI_TalonFX(Constants.Drivetrain.BACK_RIGHT_STEER_CHANNEL);
  private static final CANCoder backRightEncoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_CANCODER_CHANNEL);
  private static final SwerveModule backRight = SwerveModule.createFromDriveFalconAndSteerFalcon(backRightDriveFalcon, backRightSteerFalcon, backRightEncoder, _config, 1,0,0,1,0,0);

  private Drivetrain() {
    super(
      _config, _kinematics, _gyro, frontLeft, frontRight, backLeft, backRight);
  }
}
