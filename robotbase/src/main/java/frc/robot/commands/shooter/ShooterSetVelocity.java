package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class ShooterSetVelocity extends Command {

  private Supplier<AngularVelocity> m_velocityProvider;
  private boolean m_stopOnEnd;

  public ShooterSetVelocity(AngularVelocity velocity) {
    this(() -> velocity, true);
  }

  public ShooterSetVelocity(AngularVelocity velocity, boolean stopOnEnd) {
    this(() -> velocity, stopOnEnd);
  }

  public ShooterSetVelocity(Supplier<AngularVelocity> velocityProvider) {
    this(velocityProvider, true);
  }

  public ShooterSetVelocity(
    Supplier<AngularVelocity> velocityProvider,
    boolean stopOnEnd
  ) {
    addRequirements(Robot.shooter);
    m_velocityProvider = velocityProvider;
    m_stopOnEnd = stopOnEnd;
  }

  @Override
  public void execute() {
    Robot.shooter.setVelocity(m_velocityProvider.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_stopOnEnd) {
      Robot.shooter.stop();
    }
  }
}
