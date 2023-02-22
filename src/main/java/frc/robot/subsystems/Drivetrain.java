// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.utility.MAX_NeoSteerController_BugFix;
import frc.robot.utility.PhotonVisionWrapper;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.drivetrains.DrivetrainConfig;
import frc.team1891.common.drivetrains.SwerveDrivetrain;
import frc.team1891.common.drivetrains.swervemodules.DriveController;
import frc.team1891.common.drivetrains.swervemodules.FalconDriveController;
import frc.team1891.common.drivetrains.swervemodules.SteerController;
import frc.team1891.common.drivetrains.swervemodules.SwerveModule;
import frc.team1891.common.hardware.SimNavX;

import static frc.robot.utility.MirrorPoses.mirror;

public class Drivetrain extends SwerveDrivetrain {
  private static Drivetrain instance;
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  /**
   * Returns a new PID controller that can be used to control the position of the robot chassis in one axis.
   * @return a new {@link PIDController}
   */
  public static PIDController getTunedTranslationalPIDController() {
    return new PIDController(
      DrivetrainConstants.translationalP,
      DrivetrainConstants.translationalI,
      DrivetrainConstants.translationalD
    );
  }

  /**
   * Returns a new PID controller that can be used to control the angle of the robot chassis.
   * The output will be in radians per second, representing the desired angular velocity.
   * @return a new {@link ProfiledPIDController}
   */
  public static ProfiledPIDController getTunedProfiledPIDController() {
    ProfiledPIDController controller = new ProfiledPIDController(
      DrivetrainConstants.rotationalP,
      DrivetrainConstants.rotationalI,
      DrivetrainConstants.rotationalD,
      new TrapezoidProfile.Constraints(
        _config.chassisMaxAngularVelocityRadiansPerSecond,
        _config.chassisMaxAngularAccelerationRadiansPerSecondSquared
      )
    );
    controller.enableContinuousInput(0, 2*Math.PI);
    return controller;
  }
  /**
   * Returns a new PID controller that can be used to control the angle of the robot chassis.
   * The output will be between -1 and 1, and is meant to be fed to {@link Drivetrain#holonomicDrive(double, double, double, boolean)}.
   * @return a new {@link ProfiledPIDController}
   */
  public static ProfiledPIDController getTunedProfiledPIDControllerForHolonomicDrive() {
    ProfiledPIDController controller = new ProfiledPIDController(
      DrivetrainConstants.rotationalP,
      DrivetrainConstants.rotationalI,
      DrivetrainConstants.rotationalD,
      new TrapezoidProfile.Constraints(1, 1)
    );
    controller.enableContinuousInput(0, 2*Math.PI);
    return controller;
  }

  // Helpful object that holds information about the drivetrain.
  private static final DrivetrainConfig _config = new DrivetrainConfig(
    DrivetrainConstants.CHASSIS_MAX_VELOCITY,
    DrivetrainConstants.CHASSIS_MAX_ACCELERATION,
    DrivetrainConstants.CHASSIS_MAX_ANGULAR_VELOCITY,
    DrivetrainConstants.CHASSIS_MAX_ANGULAR_ACCELERATION,
    DrivetrainConstants.WHEEL_RADIUS_METERS,
    DrivetrainConstants.DRIVETRAIN_DRIVE_GEAR_RATIO,
    2048
  );

  // private static final NavX _gyro = new NavX();
  private static final SimNavX _gyro = new SimNavX();

  private final PhotonVisionWrapper photonVision;

  // TODO: Fix gear ratios
  // Objects to hold module related things, such as motors, and motor wrappers (drive and steer controllers)
  private static final WPI_TalonFX frontLeftDriveFalcon = new WPI_TalonFX(FrontLeft.DRIVE_CHANNEL);
  private static final DriveController frontLeftDriveController = new FalconDriveController(frontLeftDriveFalcon, _config);
  private static final SteerController frontLeftSteerController = new MAX_NeoSteerController_BugFix(FrontLeft.STEER_CHANNEL, FrontLeft.ENCODER_OFFSET_RADIANS, 1, 0, 0, 0);
  private static final SwerveModule frontLeft = new SwerveModule(frontLeftDriveController, frontLeftSteerController);
  private static final WPI_TalonFX frontRightDriveFalcon = new WPI_TalonFX(FrontRight.DRIVE_CHANNEL);
  private static final DriveController frontRightDriveController = new FalconDriveController(frontRightDriveFalcon, _config);
  private static final SteerController frontRightSteerController = new MAX_NeoSteerController_BugFix(FrontRight.STEER_CHANNEL, FrontRight.ENCODER_OFFSET_RADIANS, 1, 0, 0, 0);
  private static final SwerveModule frontRight = new SwerveModule(frontRightDriveController, frontRightSteerController);
  private static final WPI_TalonFX backLeftDriveFalcon = new WPI_TalonFX(BackLeft.DRIVE_CHANNEL);
  private static final DriveController backLeftDriveController = new FalconDriveController(backLeftDriveFalcon, _config);
  private static final SteerController backLeftSteerController = new MAX_NeoSteerController_BugFix(BackLeft.STEER_CHANNEL, BackLeft.ENCODER_OFFSET_RADIANS, 1, 0, 0, 0);
  private static final SwerveModule backLeft = new SwerveModule(backLeftDriveController, backLeftSteerController);
  private static final WPI_TalonFX backRightDriveFalcon = new WPI_TalonFX(BackRight.DRIVE_CHANNEL);
  private static final DriveController backRightDriveController = new FalconDriveController(backRightDriveFalcon, _config);
  private static final SteerController backRightSteerController = new MAX_NeoSteerController_BugFix(BackRight.STEER_CHANNEL, BackRight.ENCODER_OFFSET_RADIANS, 1, 0, 0, 0);
  private static final SwerveModule backRight = new SwerveModule(backRightDriveController, backRightSteerController);

  private Drivetrain() {
    // Call super to initialize this Drivetrain as a SwerveDrivetrain, allowing the super class to handle important features.
    super(
      _config, 
      DrivetrainConstants.WHEEL_BASE_WIDTH_METERS,
      DrivetrainConstants.WHEEL_BASE_LENGTH_METERS,
      _gyro,
      frontLeft,
      frontRight,
      backLeft,
      backRight,
      DrivetrainConstants.MODULE_MAX_VELOCITY
    );

    gyro.reset();

    // Configure all the motors.  Basically they all need to be reset, to make sure there aren't any unexpected settings saved.
    configDriveMotor(frontLeftDriveFalcon);
    configDriveMotor(frontRightDriveFalcon);
    configDriveMotor(backLeftDriveFalcon);
    configDriveMotor(backRightDriveFalcon);

    photonVision = PhotonVisionWrapper.getInstance();

    // Add some SmartDashboard settings
    SmartDashboard.putBoolean("showPhotonEstimate", true);

    if (Robot.isSimulation()) {
      LazyDashboard.addNumber("Drivetrain/xSpeed (Meters per Second)", 10, () -> simSpeeds.vxMetersPerSecond);
      LazyDashboard.addNumber("Drivetrain/ySpeed (Meters per Second)", 10, () -> simSpeeds.vyMetersPerSecond);
      LazyDashboard.addNumber("Drivetrain/omegaSpeed (Radians per Second)", 10, () -> simSpeeds.omegaRadiansPerSecond);
    }
    configureSmartDashboard();

    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      poseEstimator.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), mirror(getPose2d()));
    }
  }

  private static void configDriveMotor(WPI_TalonFX driveMotor) {
    driveMotor.configFactoryDefault();
    driveMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void holonomicDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed *= config.chassisMaxVelocityMetersPerSecond;
    ySpeed *= config.chassisMaxVelocityMetersPerSecond;
    rot *= config.chassisMaxAngularVelocityRadiansPerSecond;

    fromChassisSpeeds(
      fieldRelative?
            // If the robot is on the red alliance, the field oriented drive needs to change directions.
            (DriverStation.getAlliance().equals(Alliance.Blue)?
              ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose2d().getRotation())
            :
              ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getPose2d().getRotation().rotateBy(Rotation2d.fromDegrees(180))))
        :
            new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
  }

  /**
   * Returns the measurement of the gyro as a {@link Rotation3d}.
   * 
   * This measurement will not change if the robot's pose is reset.
   * However it will if the gyro itself is reset.
   * @return gyro measurements
   */
  public Rotation3d getGyroMeasurement() {
    // Certain axes of the gyro are inverted compared to the conventional Rotation3d.
    // Rotation3d assumes counterclockwise about each axis is positive.
    return new Rotation3d(Units.degreesToRadians(-gyro.getRoll()), Units.degreesToRadians(gyro.getPitch()), Units.degreesToRadians(-gyro.getYaw()));
  }

  @Override
  public void updateOdometry() {
    super.updateOdometry();

    // Handle vision processing and pose estimation
    if (Robot.isReal()) {
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
  }

  private ChassisSpeeds simSpeeds = new ChassisSpeeds();

  @Override
  public void simulationPeriodic() {
    // Calculate the theoretical chassis speed based on the desired position of the modules
    simSpeeds = kinematics.toChassisSpeeds(
      frontLeft.getDesiredSwerveModuleState(),
      frontRight.getDesiredSwerveModuleState(),
      backLeft.getDesiredSwerveModuleState(),
      backRight.getDesiredSwerveModuleState()
    );

    // update pose estimation and gyro readings according to theoretical chassis speeds
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
