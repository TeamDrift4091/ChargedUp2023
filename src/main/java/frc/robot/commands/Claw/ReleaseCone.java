package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ReleaseCone extends CommandBase {
    private final Claw claw;

    public ReleaseCone(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.setSolenoidPositions(Value.kReverse);
    }
}