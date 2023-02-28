// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autonomous.AutonomousCommandManager;
import frc.robot.commands.drivetrain.*;
import frc.robot.subsystems.*;
import frc.robot.utility.SmartHolonomicTrajectoryCommandGenerator;

import static frc.robot.utility.MirrorPoses.mirror;

public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  // private final Arm arm = new Arm();
  // private final Claw claw = new Claw();

  // Controllers
  private final XboxController controller = new XboxController(1) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
    };
  };

  // Buttons and triggers (These are how we schedule commands).

  // private final JoystickRotation2d rightStickRotation = new JoystickRotation2d(() -> -controller.getRightY(), () -> -controller.getRightX());
  private final JoystickButton testDrivetrian = new JoystickButton(controller, XboxController.Button.kA.value);
  private final JoystickButton toZero = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  private final JoystickButton resetOdometry = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  // JoystickButton orbitDrive = new JoystickButton(controller, 2);
  // JoystickButton squareAlign = new JoystickButton(controller, 3);

  // private final Trigger raiseArm = new POVTrigger(controller, POV.NORTH);
  // private final Trigger lowerArm = new POVTrigger(controller, POV.SOUTH);
  // private final Trigger extendArm = new POVTrigger(controller, POV.EAST);
  // private final Trigger retractArm = new POVTrigger(controller, POV.WEST);

  // private final Trigger scoreNearestHigh = new POVTrigger(controller, POV.NORTH);
  // private final Trigger scoreNearestLow = new POVTrigger(controller, POV.SOUTH);
  // private final Trigger scoreNearestMid = new POVTrigger(controller, POV.EAST);
  // private GameObject gameObjectMode = GameObject.CUBE;
  // private final Trigger toggleObjectType = new POVTrigger(controller, POV.WEST);

  private final JoystickButton toCommunity = new JoystickButton(controller, 4);
  private final JoystickButton toLoadingStation = new JoystickButton(controller, 5);

  public RobotContainer() {
    // Connects the buttons and triggers to commands
    configureBindings();
    // Loads the autonomous chooser with all of the available autonomous routines.
    // I'm doing this on a seperate thread because loading trajectories can take a lot of time.
    Thread loadAutoThread = new Thread(() -> {
      long start = System.currentTimeMillis();
      AutonomousCommandManager.load();
      System.out.printf("AUTO: Autonomous commands loaded in %.3f seconds.\n", (System.currentTimeMillis() - start)/1000.);
    }, "AutonomousCommandManager.load();");
    loadAutoThread.run();
  }

  private void configureBindings() {
    // Whenever not told to do something else, the drivetrian will run JoystickDrive.
    drivetrain.setDefaultCommand(
      new JoystickDrive(
        drivetrain,
        () -> controller.getLeftY(),
        () -> controller.getLeftX(),
        () -> controller.getRightX()
      )
    );

    // // Whenever not told to do something else, the arm will go back to it's "Zero" position.
    // arm.setDefaultCommand(new Retract(arm));

    resetOdometry.onTrue(new InstantCommand(() -> {
      drivetrain.resetOdometry();
    }));

    testDrivetrian.onTrue(new DrivetrainTest(drivetrain));
    toZero.whileTrue(new DriveToPose(drivetrain, () -> {
      if (Robot.isRedAlliance()) {
        return mirror(new Pose2d(0,0, Rotation2d.fromDegrees(180)));
      } else {
        return new Pose2d(0,0, Rotation2d.fromDegrees(180));
      }
    }));

    toCommunity.whileTrue(SmartHolonomicTrajectoryCommandGenerator.toCommunityZone(drivetrain));
    toLoadingStation.whileTrue(SmartHolonomicTrajectoryCommandGenerator.toLoadingStation(drivetrain));

    // raiseArm.whileTrue(new RunCommand(() -> arm.toPosition(new Translation2d(arm.getArmExtensionDistance(), arm.getArmAngle().rotateBy(Rotation2d.fromDegrees(1))).plus(new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND)))));
    // lowerArm.whileTrue(new RunCommand(() -> arm.toPosition(new Translation2d(arm.getArmExtensionDistance(), arm.getArmAngle().rotateBy(Rotation2d.fromDegrees(-1))).plus(new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND)))));
    // extendArm.whileTrue(new RunCommand(() -> arm.toPosition(new Translation2d(arm.getArmExtensionDistance()+.01, arm.getArmAngle()).plus(new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND)))));
    // retractArm.whileTrue(new RunCommand(() -> arm.toPosition(new Translation2d(arm.getArmExtensionDistance()-.01, arm.getArmAngle()).plus(new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND)))));

    // scoreNearestHigh.whileTrue(new DriveToAndScore(drivetrain, arm, claw, () -> gameObjectMode, ScoringLevel.HIGH));
    // scoreNearestLow.whileTrue(new DriveToAndScore(drivetrain, arm, claw, () -> gameObjectMode, ScoringLevel.LOW));
    // scoreNearestMid.whileTrue(new DriveToAndScore(drivetrain, arm, claw, () -> gameObjectMode, ScoringLevel.MEDIUM));
    // toggleObjectType.onTrue(new InstantCommand(() -> {
    //   if (gameObjectMode.equals(GameObject.CONE)) {
    //     gameObjectMode = GameObject.CUBE;
    //   } else {
    //     gameObjectMode = GameObject.CONE;
    //   }
    // }));
  }

  // This method runs at the beginning of the match to determine what command runs in autonomous.
  public Command getAutonomousCommand() {
    // Post to SmartDashboard that the command has started
    SmartDashboard.putBoolean("Autonomous Finished", false);
    // We need this in order to avoid a crash when running the same command twice. In a match this would never happen
    // but it's necessary for testing.
    CommandScheduler.getInstance().clearComposedCommands();
    // Returns the selected command from AutonomousCommandManager and appends a simple instant command that tells
    // SmartDashboard the command finished.
    return AutonomousCommandManager.getSelected().andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
  } 
}
