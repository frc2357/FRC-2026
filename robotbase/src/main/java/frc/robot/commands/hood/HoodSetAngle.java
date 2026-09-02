package frc.robot.commands.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class HoodSetAngle extends Command {

  private Supplier<Angle> m_angleSupplier;

  public HoodSetAngle(Angle angle) {
    this(() -> angle);
  }

  public HoodSetAngle(Supplier<Angle> angleSupplier) {
    addRequirements(Robot.hood);
    m_angleSupplier = angleSupplier;
  }

  @Override
  public void execute() {
    Robot.hood.setAngle(m_angleSupplier.get());
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
