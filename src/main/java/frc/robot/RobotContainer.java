// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.AutonomousCommandManager;
import frc.robot.commands.autonomous.DriveToAndScore;
import frc.robot.commands.autonomous.ScoringLocationManager.ScoringLevel;
import frc.robot.commands.claw.*;
import frc.robot.commands.clawjoint.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.subsystems.*;
import frc.robot.utility.PhotonVisionWrapper;
import frc.team1891.common.LazyDashboard;

public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final ClawJoint clawJoint = ClawJoint.getInstance();
  private final Claw claw = Claw.getInstance();

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
 
   //Triggers and button bindings; PS4
  private final Trigger resetOdometry = new JoystickButton(ps4Controller, PS4Controller.Button.kOptions.value);

  private final Trigger scoreLow = new JoystickButton(ps4Controller, PS4Controller.Button.kCross.value);
  private final Trigger scoreMid = new JoystickButton(ps4Controller, PS4Controller.Button.kCircle.value);
  private final Trigger scoreHigh = new JoystickButton(ps4Controller, PS4Controller.Button.kTriangle.value);

  // private final Trigger intakeDown = new Trigger(() -> clawDown);
  // private final Trigger toggleIntakeDeployment = new JoystickButton(ps4Controller, XboxController.Button.kLeftBumper.value);

  public RobotContainer() {
    // TODO: Disabling this only until we install the camera on the robot
    PhotonVisionWrapper.getInstance().disable();
    // Connects the buttons and triggers to commands
    configureBindings();
    // LazyDashboard.addBoolean("clawDown", () -> clawDown);
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
 // private void configureBindings() {
    // DEFAULT COMMANDS
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
    private void configureBindings() {
      // DEFAULT COMMANDS
      // Whenever not told to do something else, the drivetrian will run JoystickDrive.
      drivetrain.setDefaultCommand(
        new JoystickDrive(
          drivetrain,
          () -> ps4Controller.getLeftY(),
          () -> ps4Controller.getLeftX(),
          () -> ps4Controller.getRightX()
        )
      );

    clawJoint.setDefaultCommand(
      new ManualControl(
        clawJoint,
        () -> ps4Controller.getPOV() == 0,
        () -> ps4Controller.getPOV() == 180
      )
    );

    resetOdometry.onTrue(new InstantCommand(() -> {
      System.out.println("reset gyro.");
      drivetrain.resetGyro();
    }));

    // scoreLow.whileTrue(new DriveToAndScore(drivetrain, claw, clawJoint, ScoringLevel.HYBRID));
    // scoreMid.whileTrue(new DriveToAndScore(drivetrain, claw, clawJoint, ScoringLevel.MID));
    // scoreHigh.whileTrue(new DriveToAndScore(drivetrain, claw, clawJoint, ScoringLevel.HIGH));
    scoreLow.onTrue(new Shoot(claw, .12).withTimeout(.4));
    scoreMid.onTrue(new Shoot(claw, .2).withTimeout(.4));
    scoreHigh.onTrue(new Shoot(claw, .25).withTimeout(.4));
    // shootSimple.whileTrue(new Shoot(claw, .3));
    // intake.whileTrue(new Intake(claw));

    // intakeDown.whileTrue(ClawToAngle.intake(clawJoint));

    // // toggleIntakeDeployment.onTrue(new InstantCommand(() -> {
    // //   clawDown = !clawDown;
    // // }));

    // intakeDown.whileTrue(new RunCommand(() -> {
    //   clawJoint.drive(.3);
    // }, clawJoint));
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
    return AutonomousCommandManager.getSelected().andThen(() -> SmartDashboard.putBoolean("Autonomous Finished", true));
  } 
}
