package frc.robot.commands.tempsubsystem;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class IntakePivotAxis extends Command {

  private Supplier<Dimensionless> m_axis;

  public IntakePivotAxis(Supplier<Dimensionless> axis) {
    m_axis = axis;
    addRequirements(Robot.intakePivot);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("pivoting", true);
  }

  @Override
  public void execute() {
    Dimensionless axisValue = m_axis.get();
    SmartDashboard.putNumber("returned value", axisValue.in(Value));
    Robot.intakePivot.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intakePivot.stop();
    SmartDashboard.putBoolean("pivoting", false);
  }
}
