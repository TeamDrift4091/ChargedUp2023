// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.LowerClaw;
import frc.robot.commands.arm.RaiseClaw;
import frc.robot.commands.autonomous.AutonomousCommandManager;
import frc.robot.commands.claw.ToggleGrip;
import frc.robot.commands.drivetrain.*;
import frc.robot.subsystems.*;
import frc.team1891.common.control.AxisTrigger;
import frc.team1891.common.control.X52ProfessionalHOTAS;

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

  
  private final XboxController xboxController = new XboxController(0) {
    public double getRawAxis(int axis) {
      return MathUtil.applyDeadband(super.getRawAxis(axis), .1); // Apply a deadband to all axis to eliminate noise when it should read 0.
    };
  };
 
  private Trigger resetOdometry;

  private Trigger raiseArm;
  private Trigger lowerArm;
  private Trigger extendArm;
  private Trigger retractArm;

  private Trigger toggleClaw;

  private Trigger raiseClaw;
  private Trigger lowerClaw;

  private Trigger autoChargeStation;

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


    xBoxControls();
    // flightSimControls();


    resetOdometry.onTrue(new InstantCommand(() -> {
      if (Robot.isBlueAlliance()) {
        drivetrain.resetOdometry();
      } else {
        drivetrain.resetOdometry(mirror(new Pose2d()));
      }
    }));

    // TODO: negative is up
    raiseArm.whileTrue(new RunCommand(() -> arm.setShoulder(ControlMode.PercentOutput, -0.4), arm) {
      @Override
      public void end(boolean interrupted) {
          arm.setShoulder(ControlMode.PercentOutput, 0);
      }
    });

    lowerArm.whileTrue(new RunCommand(() -> arm.setShoulder(ControlMode.PercentOutput, 0.4), arm) {
      @Override
      public void end(boolean interrupted) {
          arm.setShoulder(ControlMode.PercentOutput, 0);
      }
    });


    // TODO: Retract is extend
    extendArm.whileTrue(new RunCommand(() -> arm.setExtension(ControlMode.PercentOutput, -1.0), arm) {
      @Override
      public void end(boolean interrupted) {
          arm.setExtension(ControlMode.PercentOutput, 0);
      }
    });

    retractArm.whileTrue(new RunCommand(() -> arm.setExtension(ControlMode.PercentOutput, 1.0), arm) {
      @Override
      public void end(boolean interrupted) {
          arm.setExtension(ControlMode.PercentOutput, 0);
      }
    });


    raiseClaw.whileTrue(new RaiseClaw(arm, .2));
    lowerClaw.whileTrue(new LowerClaw(arm, -.2));

    toggleClaw.onTrue(new ToggleGrip(claw));

    autoChargeStation.whileTrue(new BalanceOnChargingStation(drivetrain));
  }

  @SuppressWarnings("unused")
  private void xBoxControls() {
        // Whenever not told to do something else, the drivetrian will run JoystickDrive.
        drivetrain.setDefaultCommand(
          new JoystickDrive(
            drivetrain,
            () -> xboxController.getLeftY(),
            () -> xboxController.getLeftX(),
            () -> xboxController.getRightX()
          )
        );


    toggleClaw = new AxisTrigger(xboxController, XboxController.Axis.kRightTrigger.value);

    raiseClaw = new JoystickButton(xboxController, XboxController.Button.kY.value);
    lowerClaw = new JoystickButton(xboxController, XboxController.Button.kA.value);

    resetOdometry = new JoystickButton(xboxController, XboxController.Button.kStart.value);

    raiseArm = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
    lowerArm = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    extendArm = new JoystickButton(xboxController, XboxController.Button.kB.value);
    retractArm = new JoystickButton(xboxController, XboxController.Button.kX.value);
  }

  @SuppressWarnings("unused")
  private void flightSimControls() {
    // Whenever not told to do something else, the drivetrian will run JoystickDrive.
    // drivetrain.setDefaultCommand(
    //   new JoystickDrive(
    //     drivetrain,
    //     () -> flightController.getJoystickY(),
    //     () -> flightController.getJoystickX(),
    //     () -> flightController.getJoystickZ()
    //   )
    // );



    // resetOdometry = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.Fire.value);

    // raiseArm = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.JoystickBlackPOVUp.value);
    // lowerArm = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.JoystickBlackPOVDown.value);
    // extendArm = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.JoystickBlackPOVLeft.value);
    // retractArm = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.JoystickBlackPOVRight.value);

    autoChargeStation = new JoystickButton(flightController, X52ProfessionalHOTAS.Button.D.value);
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
