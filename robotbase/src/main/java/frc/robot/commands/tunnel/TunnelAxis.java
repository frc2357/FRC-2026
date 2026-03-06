package frc.robot.commands.tunnel;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class TunnelAxis extends Command {

  private Supplier<Dimensionless> m_axis;

  public TunnelAxis(Supplier<Dimensionless> axis) {
    m_axis = axis;
    addRequirements(Robot.tunnel);
  }

  @Override
  public void execute() {
    Dimensionless axisValue = m_axis.get();
    Robot.floor.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.floor.stop();
  }
}
