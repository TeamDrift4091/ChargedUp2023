package frc.robot.commands.claw;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class GrabCube extends CommandBase {
    private final Claw claw;
    private final double speed;

    public GrabCube(Claw claw, double speed) {
        this.claw = claw;
        this.speed = speed;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.setClawSpeed(speed);
    }
}