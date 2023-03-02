package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

public class OpenClaw extends InstantCommand {
    private Claw claw;
    
    public OpenClaw(Claw claw) {
        this.claw = claw;
    }
    
    @Override
    public void execute() {
        claw.openGrip();
    }
}
