package frc.robot.commands.feeder;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class FeederSetSpeed extends Command {

  private Supplier<Dimensionless> m_dutyCycleProvider;

  public FeederSetSpeed(Dimensionless dutyCycle) {
    this(() -> dutyCycle);
  }

  public FeederSetSpeed(Supplier<Dimensionless> dutyCycleProvider) {
    addRequirements(Robot.feeder);
    m_dutyCycleProvider = dutyCycleProvider;
  }

  @Override
  public void execute() {
    Robot.feeder.set(m_dutyCycleProvider.get());
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
