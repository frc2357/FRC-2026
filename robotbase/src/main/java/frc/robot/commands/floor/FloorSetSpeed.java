package frc.robot.commands.floor;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.function.Supplier;

public class FloorSetSpeed extends Command {

  private Supplier<Dimensionless> m_speed;

  public FloorSetSpeed(Dimensionless speed) {
    this(() -> speed);
  }

  public FloorSetSpeed(Supplier<Dimensionless> speed) {
    addRequirements(Robot.floor);
    m_speed = speed;
  }

  @Override
  public void execute() {
    Robot.floor.setSpeed(m_speed.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.floor.stop();
  }
}
