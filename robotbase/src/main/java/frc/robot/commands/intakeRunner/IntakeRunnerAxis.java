package frc.robot.commands.intakeRunner;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class IntakeRunnerAxis extends Command {

  private Supplier<Dimensionless> m_axis;

  public IntakeRunnerAxis(Supplier<Dimensionless> axis) {
    m_axis = axis;
    addRequirements(Robot.intake);
  }

  @Override
  public void execute() {
    Dimensionless axisValue = m_axis.get();
    Robot.intake.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
