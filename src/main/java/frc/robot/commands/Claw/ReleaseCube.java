package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ReleaseCube extends CommandBase {
    private final Claw claw;
    private final double speed;

    public ReleaseCube(Claw claw, double speed) {
        this.claw = claw;
        this.speed = speed;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.setClawSpeed(-speed);
    }
}