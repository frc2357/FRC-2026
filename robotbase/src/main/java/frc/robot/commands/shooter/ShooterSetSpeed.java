package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class ShooterSetSpeed extends Command {

  private Supplier<Dimensionless> m_dutyCycleProvider;

  public ShooterSetSpeed(Dimensionless dutyCycle) {
    this(() -> dutyCycle);
  }

  public ShooterSetSpeed(Supplier<Dimensionless> dutyCycleProvider) {
    addRequirements(Robot.shooter);
    m_dutyCycleProvider = dutyCycleProvider;
  }

  @Override
  public void execute() {
    Robot.shooter.set(m_dutyCycleProvider.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stop();
  }
}
