package frc.robot.commands.feeder;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class FeederSetSpeed extends Command {

  private Supplier<Dimensionless> m_speed;

  public FeederSetSpeed(Dimensionless speed) {
    this(() -> speed);
  }

  public FeederSetSpeed(Supplier<Dimensionless> speed) {
    m_speed = speed;
    addRequirements(Robot.feeder);
  }

  @Override
  public void execute() {
    Robot.feeder.setSpeed(m_speed.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.feeder.stop();
  }
}
