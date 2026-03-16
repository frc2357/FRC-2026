package frc.robot.commands.kicker;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class KickerSetSpeed extends Command {

  private Supplier<Dimensionless> m_speed;

  public KickerSetSpeed(Dimensionless speed) {
    this(() -> speed);
  }

  public KickerSetSpeed(Supplier<Dimensionless> speed) {
    m_speed = speed;
    addRequirements(Robot.kicker);
  }

  @Override
  public void execute() {
    Robot.kicker.setSpeed(m_speed.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.kicker.stop();
  }
}
