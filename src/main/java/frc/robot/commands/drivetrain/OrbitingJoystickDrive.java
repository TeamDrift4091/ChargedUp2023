// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class OrbitingJoystickDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> orbitPoint;
  private final DoubleSupplier forward, strafe;

  private final ProfiledPIDController angleController;
  
  public OrbitingJoystickDrive(Drivetrain drivetrain, Supplier<Pose2d> orbitPoint, DoubleSupplier forward, DoubleSupplier strafe) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.orbitPoint = orbitPoint;
    this.forward = forward;
    this.strafe = strafe;

    // TODO: Tune PID
    // Theoretically this should be the same PID as fed to trajectories.
    angleController = new ProfiledPIDController(.03, 0, 0, 
      new TrapezoidProfile.Constraints(
        drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond,
        drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared
      )
    );
    angleController.enableContinuousInput(0, 2*Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setGoal(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double f = MathUtil.applyDeadband(forward.getAsDouble(), JoystickDrive.DEADBAND);
    double s = MathUtil.applyDeadband(strafe.getAsDouble(), JoystickDrive.DEADBAND);

    Pose2d target = orbitPoint.get();
    Pose2d currentPose = drivetrain.getPose2d();
    Rotation2d angleDeltaFromTarget = target.relativeTo(currentPose).getTranslation().getAngle();

    double twist = -angleController.calculate(angleDeltaFromTarget.getRadians());
    double clampValue = .8;
    double t = MathUtil.clamp(twist, -clampValue, clampValue);

    drivetrain.holonomicDrive(f, s, t, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
