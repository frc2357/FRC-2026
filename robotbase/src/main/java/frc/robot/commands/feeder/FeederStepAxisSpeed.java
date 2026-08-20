package frc.robot.commands.feeder;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class FeederStepAxisSpeed extends Command {

  private Supplier<Dimensionless> m_axis;

  public FeederStepAxisSpeed(Supplier<Dimensionless> axis) {
    addRequirements(Robot.feeder);
    m_axis = axis;
  }

  @Override
  public void execute() {
    Robot.feeder.stepAxisSpeed(m_axis.get());
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
