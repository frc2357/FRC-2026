package frc.robot.commands.feeder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class FeederSetVelocity extends Command {

  private Supplier<AngularVelocity> m_velocityProvider;
  private boolean m_stopOnEnd;

  public FeederSetVelocity(AngularVelocity velocity) {
    this(() -> velocity, true);
  }

  public FeederSetVelocity(AngularVelocity velocity, boolean stopOnEnd) {
    this(() -> velocity, stopOnEnd);
  }

  public FeederSetVelocity(Supplier<AngularVelocity> velocityProvider) {
    this(velocityProvider, true);
  }

  public FeederSetVelocity(
    Supplier<AngularVelocity> velocityProvider,
    boolean stopOnEnd
  ) {
    addRequirements(Robot.feeder);
    m_velocityProvider = velocityProvider;
    m_stopOnEnd = stopOnEnd;
  }

  @Override
  public void execute() {
    Robot.feeder.setVelocity(m_velocityProvider.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_stopOnEnd) {
      Robot.feeder.stop();
    }
  }
}
