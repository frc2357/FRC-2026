package frc.robot.commands.Intake;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeStop extends Command {

    public IntakeStop() {
        addRequirements(Robot.intake);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Robot.intake.stop();
    }
}
