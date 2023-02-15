package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class GrabCone extends CommandBase {
    private final Claw claw;

    public GrabCone(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute() { 
        claw.closeGrip();
    }
}

