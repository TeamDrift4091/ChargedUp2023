package frc.robot.commands.claw;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class Hold extends CommandBase {
    private final Claw claw;

    public Hold(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.openGrip();
    }
 

}