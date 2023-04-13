// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToPose extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final BooleanSupplier rotate180Supplier;
  private Pose2d targetPose;
  
  private final PIDController xController, yController;
  private final ProfiledPIDController angleController;

  /**
   * Creates a command that drives to the desired pose using PID.
   * 
   * The path it uses to get there may be unpredictable, but it'll be a relatively straight line.
   * 
   * The supplier will only be checked once every time the command is initialized.
   * 
   * @param drivetrain the drivetrian to control
   * @param targetPoseSupplier the pose to drive to
   */
  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
    this(drivetrain, targetPoseSupplier, () -> false);
  }
  
  /**
   * Creates a command that drives to the desired pose using PID.
   * 
   * The path it uses to get there may be unpredictable, but it'll be a relatively straight line.
   * 
   * The supplier will only be checked once every time the command is initialized.
   * 
   * @param drivetrain the drivetrian to control
   * @param targetPoseSupplier the pose to drive to
   */
  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier, BooleanSupplier rotate180Supplier) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.targetPoseSupplier = targetPoseSupplier;
    this.targetPose = new Pose2d();
    this.rotate180Supplier = rotate180Supplier;
    xController = Drivetrain.getTunedTranslationalPIDController();
    yController = Drivetrain.getTunedTranslationalPIDController();
    angleController = Drivetrain.getTunedRotationalPIDController();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = drivetrain.getPose2d();
    angleController.reset(currentPose.getRotation().getRadians());
    xController.reset();
    yController.reset();
    targetPose = targetPoseSupplier.get();
    if (rotate180Supplier.getAsBoolean()) {
      targetPose = new Pose2d(targetPose.getTranslation(), targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }
    SmartDashboard.putNumberArray("Robot (Field2d)/targetPose", new double[] {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getPose2d();
    double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());

    xFeedback = Util.cap(xFeedback, 1.5);
    yFeedback = Util.cap(yFeedback, 1.5);

    double thetaFF = angleController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    drivetrain.fromChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xFeedback, yFeedback, thetaFF, drivetrain.getPose2d().getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    SmartDashboard.putNumberArray("Robot (Field2d)/targetPose", new double[] {-100, -100, 0});
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && angleController.atGoal();
  }
}
