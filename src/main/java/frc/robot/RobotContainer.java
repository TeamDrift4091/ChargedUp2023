// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveToAndIntakeCube;
import frc.robot.commands.autonomous.AutonomousCommandManager;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.commands.claw.*;
import frc.robot.commands.clawjoint.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.leds.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.utility.MirrorPoses;
import frc.team1891.common.control.AxisTrigger; 
import frc.team1891.common.control.POVTrigger;
import frc.team1891.common.control.POVTrigger.POV;

public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final ClawJoint clawJoint = ClawJoint.getInstance();
  private final Claw claw = Claw.getInstance();
  private final LEDs leds = LEDs.getInstance();

  public static final double normalSpeed = 2.75;
  public static final double plaidSpeed = 3.1;
  private static double currentMaxVelocity = normalSpeed;

  // Controllers; xbox
  //private final XboxController xboxController = new XboxController(0) {
   // public double getRawAxis(int axis) {
     // return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
   // };
 // };
 
  // Triggers and button bindings: xbpx
 // private final Trigger resetOdometry = new JoystickButton(xboxController, XboxController.Button.kStart.value);

  //private final Trigger shootSimple = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);

  //private final Trigger scoreLow = new JoystickButton(xboxController, XboxController.Button.kA.value);
  //private final Trigger scoreMid = new JoystickButton(xboxController, XboxController.Button.kB.value);
  //private final Trigger scoreHigh = new JoystickButton(xboxController, XboxController.Button.kY.value);

  // controllers: PS4
  private final PS4Controller ps4Controller = new PS4Controller(0) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
    };
  };

  private final XboxController secondController = new XboxController(1) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
    };
  };
  
  private final Trigger alignForward = new POVTrigger(ps4Controller, POV.NORTH).or(new POVTrigger(secondController, POV.NORTH));
  private final Trigger alignReverse = new POVTrigger(ps4Controller, POV.SOUTH).or(new POVTrigger(secondController, POV.SOUTH));
  private final Trigger cancelAlignment = new AxisTrigger(ps4Controller, PS4Controller.Axis.kRightX.value, .05);
  private final Trigger leftXTrigger = new AxisTrigger(ps4Controller, PS4Controller.Axis.kLeftX.value, .05);
  private final Trigger leftYTrigger = new AxisTrigger(ps4Controller, PS4Controller.Axis.kLeftY.value, .05);
  
   //Triggers and button bindings; PS4
  private final Trigger resetOdometry = new JoystickButton(ps4Controller, PS4Controller.Button.kOptions.value);

  private final Trigger scoreLow = new JoystickButton(ps4Controller, PS4Controller.Button.kCross.value);
  private final Trigger scoreMid = new JoystickButton(ps4Controller, PS4Controller.Button.kCircle.value);
  private final Trigger scoreHigh = new JoystickButton(ps4Controller, PS4Controller.Button.kTriangle.value);
  private final Trigger shootFar = new JoystickButton(ps4Controller, PS4Controller.Button.kSquare.value);
  private final Trigger runIntake = new JoystickButton(ps4Controller, PS4Controller.Button.kR1.value);

  private final Trigger driveToCube = new JoystickButton(ps4Controller, PS4Controller.Button.kR2.value);
  // private final Trigger alignToCubeNode = new JoystickButton(ps4Controller, PS4Controller.Button.kL2.value);

  private final Trigger deployIntake = new JoystickButton(ps4Controller, PS4Controller.Button.kL1.value);

  private final Trigger chargeStationBalance = new JoystickButton(ps4Controller, PS4Controller.Button.kL3.value);

  private final Trigger resetGyroPitch = new JoystickButton(ps4Controller, PS4Controller.Button.kR3.value);

  private final Trigger setPlaidSpeed = new JoystickButton(secondController, XboxController.Button.kA.value);
  private final Trigger ledRainbow = new JoystickButton(secondController, XboxController.Button.kLeftBumper.value);

  private final Trigger lockModules = new JoystickButton(secondController, XboxController.Button.kX.value)
    .and(cancelAlignment.negate())
    .and(leftYTrigger.negate())
    .and(leftXTrigger.negate());

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
  //xbox
  private void configureBindings() {
    // DEFAULT COMMANDS
    leds.setDefaultCommand(new LEDDefaultCommand(leds));
    // Whenever not told to do something else, the drivetrian will run JoystickDrive.
   // drivetrain.setDefaultCommand(
     // new JoystickDrive(
      //  drivetrain,
      //  () -> xboxController.getLeftY(),
      //  () -> xboxController.getLeftX(),
       // () -> xboxController.getRightX()
     // )
    //);
    
    //ps4

      // DEFAULT COMMANDS
      // Whenever not told to do something else, the drivetrian will run JoystickDrive.

    drivetrain.setDefaultCommand(
      new JoystickDrive(
        drivetrain,
        () -> ps4Controller.getLeftY(),
        () -> ps4Controller.getLeftX(),
        () -> ps4Controller.getRightX()
      )
      // new RunCommand(() -> {
      //   drivetrain.fromChassisSpeeds(new ChassisSpeeds(ps4Controller.getLeftX(), 0, 0));
      // }, drivetrain)
    );

    alignForward.onTrue(new AbsoluteAngleJoystickDrive(drivetrain, 
      () -> ps4Controller.getLeftY(), 
      () -> ps4Controller.getLeftX(), 
      () -> MirrorPoses.getForwardForAlliance()));

    alignReverse.onTrue(new AbsoluteAngleJoystickDrive(drivetrain, 
      () -> ps4Controller.getLeftY(), 
      () -> ps4Controller.getLeftX(), 
      () -> MirrorPoses.getForwardForAlliance().rotateBy(Rotation2d.fromDegrees(180))));

    cancelAlignment.whileTrue(
      new JoystickDrive(
        drivetrain,
        () -> ps4Controller.getLeftY(),
        () -> ps4Controller.getLeftX(),
        () -> ps4Controller.getRightX()
      )
    );

    clawJoint.setDefaultCommand(
      // new ManualControl(
      //   clawJoint,
      //   () -> ps4Controller.getPOV() == 0,
      //   () -> ps4Controller.getPOV() == 180
      // )
      new HomeClawPosition(clawJoint)
    );

    resetOdometry.onTrue(new InstantCommand(() -> {
      System.out.println("reset gyro.");
      drivetrain.resetGyro();
    }));

    resetGyroPitch.onTrue(new InstantCommand(() -> BalanceOnChargingStationLinear.calibrateOffset()));

    // scoreLow.whileTrue(new DriveToAndScore(drivetrain, claw, clawJoint, ScoringLevel.HYBRID));
    // scoreMid.whileTrue(new DriveToAndScore(drivetrain, claw, clawJoint, ScoringLevel.MID));
    // scoreHigh.whileTrue(new DriveToAndScore(drivetrain, claw, clawJoint, ScoringLevel.HIGH));
    scoreLow.onTrue(new ParallelRaceGroup(
      new Shoot(claw, ScoringLevel.HYBRID).withTimeout(.4),
      new CustomColor(leds, 0, 50, 0)));
    scoreMid.onTrue(new ParallelRaceGroup(
      new Shoot(claw, ScoringLevel.MID).withTimeout(.4),
      new CustomColor(leds, 0, 50, 0)));
    scoreHigh.onTrue(new ParallelRaceGroup(
      new Shoot(claw, ScoringLevel.HIGH).withTimeout(.4),
      new CustomColor(leds, 0, 50, 0)));
    shootFar.onTrue(new ParallelRaceGroup(
      new Shoot(claw, .75).withTimeout(.5),
      new CustomColor(leds, 150, 75, 0)));

    // alignToCubeNode.whileTrue(new DriveToPose(drivetrain, () -> ScoringLocationManager.getNearestCubeNodeAlignment()));
    // alignToNode.whileTrue(new DriveToPose(drivetrain, () -> ScoringLocationManager.getNearestNodeAlignment()));
    // alignToCubeNode.whileTrue(new LEDLimelightFault(leds));
    // alignToNode.whileTrue(new LEDLimelightFault(leds));

    driveToCube.whileTrue(new ParallelRaceGroup(
      new DriveToAndIntakeCube(drivetrain, claw, clawJoint),
      new LEDLimelightFault(leds)
    ));

    runIntake.whileTrue(new Intake(claw));
  
    deployIntake.whileTrue(ClawToAngle.intake(clawJoint));

    chargeStationBalance.whileTrue(new BalanceOnChargingStationLinear(drivetrain));

    setPlaidSpeed.onTrue(new InstantCommand(() -> {
      currentMaxVelocity = plaidSpeed;
    }));
    setPlaidSpeed.onFalse(new InstantCommand(() -> {
      currentMaxVelocity = normalSpeed;
    }));

    ledRainbow.onTrue(new InstantCommand(() -> {
      leds.start();
      leds.setMode(LEDMode.AUTONOMOUS);
    }, leds) {
      public void end(boolean interrupted) {
        leds.setMode(LEDMode.OFF);
        leds.stop();
      };
      public boolean isFinished() {
        return false;
      };
    }.withTimeout(5));

    lockModules.whileTrue(new RunCommand(() -> {
      drivetrain.moduleXConfiguration();
    }, drivetrain));
  }

  public static double getTeleopTranslationalVelocity() {
    return currentMaxVelocity;
  }

  // This method runs at the beginning of the match to determine what command runs in autonomous.
  public Command getAutonomousCommand() {
    // Post to SmartDashboard that the command has started
    SmartDashboard.putBoolean("Autonomous Finished", false);
    // We need this in order to avoid a crash when running the same command twice. In a match this would never happen
    // but it's necessary for testing.  We only need it because were adding the .andThen with the SmartDashboard output.
    CommandScheduler.getInstance().clearComposedCommands();
    // Returns the selected command from AutonomousCommandManager and appends a simple instant command that tells
    // SmartDashboard the command finished.
    Command autoCommand = AutonomousCommandManager.getSelected();
    if (autoCommand == null) {
      autoCommand = new InstantCommand();
    }
    return autoCommand.andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
  } 
}
