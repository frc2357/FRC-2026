package frc.robot.commands.hood;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class HoodAxisSpeed extends Command {

  private Supplier<Dimensionless> m_axis;

  public HoodAxisSpeed(Supplier<Dimensionless> axis) {
    addRequirements(Robot.hood);
    m_axis = axis;
  }

  @Override
  public void execute() {
    Robot.hood.axisSpeed(m_axis.get());
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
