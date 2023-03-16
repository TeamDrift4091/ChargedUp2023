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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmToPose;
import frc.robot.commands.arm.LowerClaw;
import frc.robot.commands.arm.RaiseClaw;
import frc.robot.commands.arm.Retract;
import frc.robot.commands.autonomous.AutonomousCommandManager;
import frc.robot.commands.autonomous.DriveToAndScore;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.commands.claw.ToggleGrip;
import frc.robot.commands.drivetrain.*;
import frc.robot.subsystems.*;
import frc.robot.utility.GameObject;
import frc.robot.utility.SmartHolonomicTrajectoryCommandGenerator;
import frc.team1891.common.control.AxisTrigger;
import frc.team1891.common.control.POVTrigger;
import frc.team1891.common.control.X52ProfessionalHOTAS;
import frc.team1891.common.control.POVTrigger.Direction;
import frc.team1891.common.control.POVTrigger.POV;

import static frc.robot.utility.MirrorPoses.mirror;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Arm arm = Arm.getInstance();
  private final Claw claw = Claw.getInstance();

  // Controllers
  private final X52ProfessionalHOTAS flightController = new X52ProfessionalHOTAS(1) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
    }
    
    public double getJoystickX() {
      return super.getJoystickX() * ((super.getThrottle() - 1) / -2.);
    }

    public double getJoystickY() {
      return super.getJoystickY() * ((super.getThrottle() - 1) / -2.);
    }

    public double getJoystickZ() {
      return super.getJoystickZ() * ((super.getThrottle() - 1) / -2.);
    }
  };
  private final XboxController controller = new XboxController(0) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
    };
  };
 
  // Buttons and triggers (These are how we schedule commands).
  // private Trigger zAxis;
  // private Trigger holdNorth;
  // private Trigger holdSouth;

  private Trigger testDrivetrian;
  // private Trigger toZero;
  private Trigger resetOdometry;

  private Trigger anyPOV;
  private Trigger raiseArm;
  private Trigger lowerArm;
  private Trigger extendArm;
  private Trigger retractArm;
  private Trigger resetArm;

  private Trigger toggleClaw;

  private Trigger raiseClaw;
  private Trigger lowerClaw;

  private Trigger autoChargeStation;
 
  // private Trigger scoreNearestHigh;
  // private Trigger scoreNearestLow;
  // private Trigger scoreNearestMid;
  // private GameObject gameObjectMode = GameObject.CUBE;
  // private Trigger toggleObjectType;

  // private Trigger toCommunity;
  // private Trigger toLoadingStation;

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

    // // Whenever not told to do something else, the arm will go back to it's "Zero" position.
    // // arm.setDefaultCommand(new Retract(arm));
    // // Whenever not told to move, the arm will hold it's position
    // arm.setDefaultCommand(ArmToPose.holdPose(arm));


    // ONLY ONE OF THESE LINES SHOULD BE UNCOMMENTED WHEN DEPLOYING TO THE ROBOT
    // xBoxControls();
    flightSimControls();


    resetOdometry.onTrue(new InstantCommand(() -> {
      if (Robot.isBlueAlliance()) {
        drivetrain.resetOdometry();
      } else {
        drivetrain.resetOdometry(mirror(new Pose2d()));
      }
    }));

    // testDrivetrian.onTrue(new DrivetrainTest(drivetrain));
    // toZero.whileTrue(new DriveToPose(drivetrain, () -> {
    //   if (Robot.isRedAlliance()) {
    //     return mirror(new Pose2d(0,0, new Rotation2d()));
    //   } else {
    //     return new Pose2d(0,0, new Rotation2d());
    //   }
    // }));
     




    // toCommunity.whileTrue(SmartHolonomicTrajectoryCommandGenerator.toCommunityZone(drivetrain));
    // toLoadingStation.whileTrue(SmartHolonomicTrajectoryCommandGenerator.toLoadingStation(drivetrain));

    // raiseArm.whileTrue(new RunCommand(() -> arm.toPosition(new Translation2d(arm.getArmExtensionDistance(), arm.getArmAngle().rotateBy(Rotation2d.fromDegrees(1))).plus(new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND)))));
    // lowerArm.whileTrue(new RunCommand(() -> arm.toPosition(new Translation2d(arm.getArmExtensionDistance(), arm.getArmAngle().rotateBy(Rotation2d.fromDegrees(-1))).plus(new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND)))));
    // extendArm.whileTrue(new RunCommand(() -> arm.toPosition(new Translation2d(arm.getArmExtensionDistance()+.01, arm.getArmAngle()).plus(new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND)))));
    // retractArm.whileTrue(new RunCommand(() -> arm.toPosition(new Translation2d(arm.getArmExtensionDistance()-.01, arm.getArmAngle()).plus(new Translation2d(0, ArmConstants.SHOULDER_HEIGHT_FROM_GROUND)))));
    
    
    // anyPOV.whileTrue(
    //   new RunCommand(() -> {
    //     double desiredExtension = arm.getArmExtensionDistance();
    //     Rotation2d desiredRotation = arm.getArmAngle();
    //     if (raiseArm.getAsBoolean()) {
    //       desiredRotation.plus(Rotation2d.fromDegrees(1));
    //     } else if (lowerArm.getAsBoolean()) {
    //       desiredRotation.plus(Rotation2d.fromDegrees(-1));
    //     }
    //     if (extendArm.getAsBoolean()) {
    //       desiredExtension += .01;
    //     } else if (retractArm.getAsBoolean()) {
    //       desiredExtension -= .01;
    //     }
    //     arm.toPosition(desiredExtension, desiredRotation);
    //   }, arm)
    // );

    // TODO: negative is up
    raiseArm.whileTrue(new RunCommand(() -> arm.setShoulder(ControlMode.PercentOutput, -0.3), arm) {
      @Override
      public void end(boolean interrupted) {
          arm.setShoulder(ControlMode.PercentOutput, 0);
      }
    });

    lowerArm.whileTrue(new RunCommand(() -> arm.setShoulder(ControlMode.PercentOutput, 0.3), arm) {
      @Override
      public void end(boolean interrupted) {
          arm.setShoulder(ControlMode.PercentOutput, 0);
      }
    });


    // TODO: Retract is extend
    extendArm.whileTrue(new RunCommand(() -> arm.setExtension(ControlMode.PercentOutput, -0.6), arm) {
      @Override
      public void end(boolean interrupted) {
          arm.setExtension(ControlMode.PercentOutput, 0);
      }
    });

    retractArm.whileTrue(new RunCommand(() -> arm.setExtension(ControlMode.PercentOutput, 0.6), arm) {
      @Override
      public void end(boolean interrupted) {
          arm.setExtension(ControlMode.PercentOutput, 0);
      }
    });

    resetArm.onTrue(new InstantCommand(() -> arm.reset()));

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

    
    // toCommunity.whileTrue(SmartHolonomicTrajectoryCommandGenerator.toCommunityZone(drivetrain));
    // toLoadingStation.whileTrue(SmartHolonomicTrajectoryCommandGenerator.toLoadingStation(drivetrain));
    

    raiseClaw.whileTrue(new RaiseClaw(arm, .2));
    lowerClaw.whileTrue(new LowerClaw(arm, -.2));

    autoChargeStation.whileTrue(new BalanceOnChargingStation(drivetrain));
  }

  @SuppressWarnings("unused")
  private void xBoxControls() {
        // Whenever not told to do something else, the drivetrian will run JoystickDrive.
        drivetrain.setDefaultCommand(
          new JoystickDrive(
            drivetrain,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX()
          )
        );

    testDrivetrian = new JoystickButton(controller, XboxController.Button.kA.value);
    // toZero = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    resetOdometry = new JoystickButton(controller, XboxController.Button.kRightBumper.value);

    anyPOV = POVTrigger.anyPOV(controller);
    raiseArm = POVTrigger.asButton(controller, Direction.UP);
    lowerArm = POVTrigger.asButton(controller, Direction.DOWN);
    extendArm = POVTrigger.asButton(controller, Direction.RIGHT);
    retractArm = POVTrigger.asButton(controller, Direction.LEFT);
    resetArm = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);

    // scoreNearestHigh = new POVTrigger(controller, POV.NORTH);
    // scoreNearestLow = new POVTrigger(controller, POV.SOUTH);
    // scoreNearestMid = new POVTrigger(controller, POV.EAST);
    // toggleObjectType = new POVTrigger(controller, POV.WEST);

    // toCommunity = new JoystickButton(controller, 4);
    // toLoadingStation = new JoystickButton(controller, 5);
  }

  @SuppressWarnings("unused")
  private void flightSimControls() {
    // Whenever not told to do something else, the drivetrian will run JoystickDrive.
    drivetrain.setDefaultCommand(
      new JoystickDrive(
        drivetrain,
        () -> flightController.getJoystickY(),
        () -> flightController.getJoystickX(),
        () -> flightController.getJoystickZ()
      )
    );

    // zAxis = new AxisTrigger(flightController, X52ProfessionalHOTAS.Axis.JoystickZ.value);
    // zAxis.onTrue(
    //   new JoystickDrive(
    //     drivetrain,
    //     () -> flightController.getJoystickY(),
    //     () -> flightController.getJoystickX(),
    //     () -> flightController.getJoystickZ()
    //   )
    // );
    // holdNorth = new POVTrigger(flightController, POV.NORTH);
    // holdNorth.onTrue(new AbsoluteAngleJoystickDrive(
    //   drivetrain, 
    //   () -> flightController.getJoystickY(),
    //   () -> flightController.getJoystickX(),
    //   Robot.isBlueAlliance()?
    //     () -> new Rotation2d():
    //     () -> Rotation2d.fromDegrees(180)
    // ));

    // holdSouth = new POVTrigger(flightController, POV.SOUTH);
    // holdSouth.onTrue(new AbsoluteAngleJoystickDrive(
    //   drivetrain, 
    //   () -> flightController.getJoystickY(),
    //   () -> flightController.getJoystickX(),
    //   Robot.isBlueAlliance()?
    //   () -> Rotation2d.fromDegrees(180):
    //   () -> new Rotation2d()
    // ));

    // testDrivetrian = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.MouseButton.value);
    // toZero = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.E.value);
    resetOdometry = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.Fire.value);

    raiseArm = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.JoystickBlackPOVUp.value);
    lowerArm = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.JoystickBlackPOVDown.value);
    extendArm = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.JoystickBlackPOVLeft.value);
    retractArm = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.JoystickBlackPOVRight.value);
    anyPOV = raiseArm.or(lowerArm).or(extendArm).or(retractArm);
    resetArm = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.C.value);
    
    // toCommunity = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.T1Down.value);
    // toLoadingStation = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.T1Up.value);

    // Trigger scoreNearest = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.C.value);
    // toggleObjectType = new POVTrigger(controller, POV.WEST);

    toggleClaw = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.JoystickTriggerFirstLevel.value);
    toggleClaw.onTrue(new ToggleGrip(claw));

    raiseClaw = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.A.value);
    lowerClaw = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.B.value);

    autoChargeStation = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.T3Up.value);
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
