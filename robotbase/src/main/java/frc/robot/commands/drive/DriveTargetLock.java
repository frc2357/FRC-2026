package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import java.util.function.Supplier;

public class DriveTargetLock extends Command {

  Supplier<Dimensionless> m_x;
  Supplier<Dimensionless> m_y;

  SlewRateLimiter m_LimiterX;
  SlewRateLimiter m_LimiterY;

  public DriveTargetLock(Supplier<Dimensionless> x, Supplier<Dimensionless> y) {
    addRequirements(Robot.swerve);

    m_x = x;
    m_y = y;

    m_LimiterX = new SlewRateLimiter(SWERVE.SLEW_RATE_LIMIT);
    m_LimiterY = new SlewRateLimiter(SWERVE.SLEW_RATE_LIMIT);
  }

  @Override
  public void execute() {
    Rotation2d target = Robot.shotCalculator.getCalculatedDriveAngle();
    double m_xPercent = m_LimiterX.calculate(m_x.get().in(Value));
    double m_yPercent = m_LimiterY.calculate(m_y.get().in(Value));

    Robot.swerve.driveAtAngle(
      Constants.SWERVE.AXIS_MAX_SPEED.times(SWERVE.MAX_SPEED).times(m_yPercent),
      Constants.SWERVE.AXIS_MAX_SPEED.times(SWERVE.MAX_SPEED).times(m_xPercent),
      target
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
