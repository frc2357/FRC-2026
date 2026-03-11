package frc.robot.commands.intakerunner;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeRunnerSetSpeed extends Command {

  private Dimensionless m_speed;

  public IntakeRunnerSetSpeed(Dimensionless speed) {
    m_speed = speed;
    addRequirements(Robot.intake);
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
