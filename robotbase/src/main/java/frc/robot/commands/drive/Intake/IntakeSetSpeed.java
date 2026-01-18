public class IntakeSetSpeed  extends Command {

  private double m_speed;

  public IntakeSetSpeed(double speed) {
    addRequirements(Robot.setSpeed);
    m_speed = speed;
  }

  @Override
  public void initialize() {
    Robot.setSpeed.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.setSpeed.stop();
  }
}
