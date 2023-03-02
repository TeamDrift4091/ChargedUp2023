import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class EjectFromClaw extends CommandBase {
    private Claw claw;
    private double speed;

    public EjectFromClaw(Claw claw, double speed) {
        this.claw = claw;
        this.speed = speed;
        addRequirements(claw); // This command requires the claw subsystem
    }

    

    @Override
    public void initialize() {
        claw.startMotors(-speed); 
    }

    @Override
    protected void end() {
        claw.stopMotors(); 
        claw.openGrip(); 
    }
}
