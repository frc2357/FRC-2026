package frc.robot.commands.feeder;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class FeederAxis extends Command {

  private Supplier<Dimensionless> m_axis;

  public FeederAxis(Supplier<Dimensionless> axis) {
    m_axis = axis;
    addRequirements(Robot.feeder);
  }

  @Override
  public void execute() {
    Dimensionless axisValue = m_axis.get();
    Robot.feeder.setAxisSpeed(axisValue);
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
