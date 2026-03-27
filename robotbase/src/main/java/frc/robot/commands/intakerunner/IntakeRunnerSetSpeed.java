package frc.robot.commands.intakerunner;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class IntakeRunnerSetSpeed extends Command {

  private Supplier<Dimensionless> m_speed;

  public IntakeRunnerSetSpeed(Dimensionless speed) {
    this(() -> speed);
  }

  public IntakeRunnerSetSpeed(Supplier<Dimensionless> speed) {
    m_speed = speed;
    addRequirements(Robot.intake);
  }

  @Override
  public void execute() {
    Robot.intake.setSpeed(m_speed.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
