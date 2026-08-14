package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class ShooterStepAxisSpeed extends Command {

  private Supplier<Dimensionless> m_axis;

  public ShooterStepAxisSpeed(Supplier<Dimensionless> axis) {
    addRequirements(Robot.shooter);
    m_axis = axis;
  }

  @Override
  public void execute() {
    Robot.shooter.stepAxisSpeed(m_axis.get());
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
