package frc.robot.commands.outtake;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class OuttakeAxis extends Command {

  private Supplier<Dimensionless> m_axis;

  public OuttakeAxis(Supplier<Dimensionless> axis) {
    m_axis = axis;
    addRequirements(Robot.intake);
  }

  @Override
  public void execute() {
    Dimensionless axisValue = m_axis.get();
    Robot.outtake.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.outtake.stop();
  }
}
