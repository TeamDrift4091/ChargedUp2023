// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utility.PhotonVisionWrapper;
import frc.team1891.common.drivetrains.DrivetrainConfig;
import frc.team1891.common.drivetrains.SwerveDrivetrain;
import frc.team1891.common.drivetrains.swervemodules.DriveController;
import frc.team1891.common.drivetrains.swervemodules.FalconDriveController;
import frc.team1891.common.drivetrains.swervemodules.SDS_NeoSteerController;
import frc.team1891.common.drivetrains.swervemodules.SteerController;
import frc.team1891.common.drivetrains.swervemodules.SwerveModule;
import frc.team1891.common.hardware.SimNavX;

public class Drivetrain extends SwerveDrivetrain {
  private static Drivetrain instance;
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  private static final DrivetrainConfig _config = new DrivetrainConfig(2, 1, Math.PI, Math.PI, 2, Constants.Drivetrain.DRIVETRAIN_GEAR_RATIO, 2048);

  // private static final NavX _gyro = new NavX();
  private static final SimNavX _gyro = new SimNavX();

  private final PhotonVisionWrapper photonVision;

  // private final SwerveSim sim;

  // TODO: Fix gear ratios
  private static final WPI_TalonFX frontLeftDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.FRONT_LEFT_DRIVE_CHANNEL);
  private static final CANSparkMax frontLeftSteerNeo = new CANSparkMax(Constants.Drivetrain.FRONT_LEFT_STEER_CHANNEL, MotorType.kBrushless);
  private static final WPI_CANCoder frontLeftEncoder = new WPI_CANCoder(Constants.Drivetrain.FRONT_LEFT_CANCODER_CHANNEL);
  private static final DriveController frontLeftDriveController = new FalconDriveController(frontLeftDriveFalcon, _config);
  private static final SteerController frontLeftSteerController = new SDS_NeoSteerController(frontLeftSteerNeo, frontLeftEncoder, 150/7d);
  private static final SwerveModule frontLeft = new SwerveModule(frontLeftDriveController, frontLeftSteerController);
  private static final WPI_TalonFX frontRightDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.FRONT_RIGHT_DRIVE_CHANNEL);
  private static final CANSparkMax frontRightSteerNeo = new CANSparkMax(Constants.Drivetrain.FRONT_RIGHT_STEER_CHANNEL, MotorType.kBrushless);
  private static final WPI_CANCoder frontRightEncoder = new WPI_CANCoder(Constants.Drivetrain.FRONT_RIGHT_CANCODER_CHANNEL);
  private static final DriveController frontRightDriveController = new FalconDriveController(frontRightDriveFalcon, _config);
  private static final SteerController frontRightSteerController = new SDS_NeoSteerController(frontRightSteerNeo, frontRightEncoder, 150/7d);
  private static final SwerveModule frontRight = new SwerveModule(frontRightDriveController, frontRightSteerController);
  private static final WPI_TalonFX backLeftDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.BACK_LEFT_DRIVE_CHANNEL);
  private static final CANSparkMax backLeftSteerNeo = new CANSparkMax(Constants.Drivetrain.BACK_LEFT_STEER_CHANNEL, MotorType.kBrushless);
  private static final WPI_CANCoder backLeftEncoder = new WPI_CANCoder(Constants.Drivetrain.BACK_LEFT_CANCODER_CHANNEL);
  private static final DriveController backLeftDriveController = new FalconDriveController(backLeftDriveFalcon, _config);
  private static final SteerController backLeftSteerController = new SDS_NeoSteerController(backLeftSteerNeo, frontLeftEncoder, 150/7d);
  private static final SwerveModule backLeft = new SwerveModule(backLeftDriveController, backLeftSteerController);
  private static final WPI_TalonFX backRightDriveFalcon = new WPI_TalonFX(Constants.Drivetrain.BACK_RIGHT_DRIVE_CHANNEL);
  private static final CANSparkMax backRightSteerNeo = new CANSparkMax(Constants.Drivetrain.BACK_RIGHT_STEER_CHANNEL, MotorType.kBrushless);
  private static final WPI_CANCoder backRightEncoder = new WPI_CANCoder(Constants.Drivetrain.BACK_RIGHT_CANCODER_CHANNEL);
  private static final DriveController backRightDriveController = new FalconDriveController(backRightDriveFalcon, _config);
  private static final SteerController backRightSteerController = new SDS_NeoSteerController(backRightSteerNeo, backRightEncoder, 150/7d);
  private static final SwerveModule backRight = new SwerveModule(backRightDriveController, backRightSteerController);

  private Drivetrain() {
    super(
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
    // configSteerMotor(frontLeftSteerFalcon);
    // configSteerMotor(frontRightSteerFalcon);
    // configSteerMotor(backLeftSteerFalcon);
    // configSteerMotor(backRightSteerFalcon);
    configCANCoder(frontLeftEncoder, Constants.Drivetrain.FRONT_LEFT_ENCODER_OFFSET);
    configCANCoder(frontRightEncoder, Constants.Drivetrain.FRONT_RIGHT_ENCODER_OFFSET);
    configCANCoder(backLeftEncoder, Constants.Drivetrain.BACK_LEFT_ENCODER_OFFSET);
    configCANCoder(backRightEncoder, Constants.Drivetrain.BACK_RIGHT_ENCODER_OFFSET);

    photonVision = new PhotonVisionWrapper();

    SmartDashboard.putBoolean("showPhotonEstimate", true);
    configureSmartDashboard();
  }

  private static void configDriveMotor(WPI_TalonFX driveMotor) {
    driveMotor.configFactoryDefault();
    driveMotor.setNeutralMode(NeutralMode.Brake);
  }

  // private static void configSteerMotor(WPI_TalonFX steerMotor) {
  //   steerMotor.configFactoryDefault();
  //   steerMotor.setNeutralMode(NeutralMode.Brake);
  // }

  private static void configCANCoder(WPI_CANCoder encoder, double encoderOffsetDegrees) {
    encoder.configFactoryDefault();
    // encoder.configMagnetOffset(encoderOffsetDegrees);
  }

  @Override
  public void updateOdometry() {
    super.updateOdometry();

    Optional<EstimatedRobotPose> result =
      photonVision.getEstimatedGlobalPose(getPose2d());
  
    if (result.isPresent()){
      EstimatedRobotPose camPose = result.get();
      poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      if (SmartDashboard.getBoolean("showPhotonEstimate", false)) {
        field.getObject("photonEstimate").setPose(camPose.estimatedPose.toPose2d());
      }
    } else {
      if (SmartDashboard.getBoolean("showPhotonEstimate", false)) {
        // move it way off the screen to make it disappear
        field.getObject("photonEstimate").setPose(new Pose2d(-100, -100, new Rotation2d()));
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds simSpeeds = kinematics.toChassisSpeeds(
      frontLeft.getDesiredSwerveModuleState(),
      frontRight.getDesiredSwerveModuleState(),
      backLeft.getDesiredSwerveModuleState(),
      backRight.getDesiredSwerveModuleState()
    );

    _gyro.setRadians(_gyro.getRadians() - simSpeeds.omegaRadiansPerSecond * .02);
    Pose2d newPose = poseEstimator.getEstimatedPosition().plus(
      new Transform2d(
        new Translation2d(
          simSpeeds.vxMetersPerSecond * .02,
          simSpeeds.vyMetersPerSecond * .02
        ),
        new Rotation2d()
      )
    );

    poseEstimator.resetPosition(_gyro.getRotation2d(), getSwerveModulePositions(), newPose);
  }
}
