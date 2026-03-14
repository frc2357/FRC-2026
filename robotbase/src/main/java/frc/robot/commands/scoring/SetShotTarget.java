package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class SetShotTarget extends Command {

  private Translation2d m_previousTarget;
  private Supplier<Translation2d> m_targetSupplier;

  public SetShotTarget(Translation2d target) {
    m_targetSupplier = () -> target;
  }

  public SetShotTarget(Supplier<Translation2d> targetSupplier) {
    m_targetSupplier = targetSupplier;
  }

  @Override
  public void initialize() {
    m_previousTarget = Robot.scoreCalculator.getShotTarget();
  }

  @Override
  public void execute() {
    if (m_targetSupplier != null) {
      Robot.scoreCalculator.setShotTarget(m_targetSupplier.get());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.scoreCalculator.setShotTarget(m_previousTarget);
  }
}
