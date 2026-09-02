package frc.robot.commands.hood;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class HoodSetSpeed extends Command {

  private Supplier<Dimensionless> m_dutyCycleSupplier;

  public HoodSetSpeed(Dimensionless dutyCycle) {
    this(() -> dutyCycle);
  }

  public HoodSetSpeed(Supplier<Dimensionless> dutyCycleSupplier) {
    addRequirements(Robot.hood);
    m_dutyCycleSupplier = dutyCycleSupplier;
  }

  @Override
  public void execute() {
    Robot.hood.set(m_dutyCycleSupplier.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.hood.stop();
  }
}
