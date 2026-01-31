package frc.robot.commands.outtake;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class OuttakeSetSpeed extends Command {

  private Dimensionless m_speed;

  public OuttakeSetSpeed(Dimensionless speed) {
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
