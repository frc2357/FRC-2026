package frc.robot.commands.intake;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeSetSpeed extends Command {

  private Dimensionless m_speed;

  public IntakeSetSpeed(Dimensionless speed) {
    addRequirements(Robot.intake);
    m_speed = speed;
  }

  @Override
  public void initialize() {
    Robot.intake.setSpeed(m_speed);
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
