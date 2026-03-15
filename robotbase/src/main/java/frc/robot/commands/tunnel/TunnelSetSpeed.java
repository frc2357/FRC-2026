package frc.robot.commands.tunnel;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class TunnelSetSpeed extends Command {

  private Supplier<Dimensionless> m_speed;

  public TunnelSetSpeed(Dimensionless speed) {
    this(() -> speed);
  }

  public TunnelSetSpeed(Supplier<Dimensionless> speed) {
    addRequirements(Robot.tunnel);
    m_speed = speed;
  }

  @Override
  public void execute() {
    Robot.tunnel.setSpeed(m_speed.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.tunnel.stop();
  }
}
