package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class GrabCube extends CommandBase {
    private Claw claw;
    private double speed;

    public GrabCube(Claw claw, double speed) {
        this.claw = claw;
        this.speed = speed;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.setMotorSpeed(speed, speed);
    }

    @Override
    public void end(boolean interrupted) {
        claw.stopMotors(); 
        claw.closeGrip(); 
    }
}