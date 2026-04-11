package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import java.util.function.Supplier;

public class DefaultDrive extends Command {

  Supplier<Dimensionless> m_x;
  Supplier<Dimensionless> m_y;
  Supplier<Dimensionless> m_rotation;

  SlewRateLimiter m_LimiterX;
  SlewRateLimiter m_LimiterY;
  SlewRateLimiter m_LimiterRotation;

  public DefaultDrive(
    Supplier<Dimensionless> x,
    Supplier<Dimensionless> y,
    Supplier<Dimensionless> rotation
  ) {
    addRequirements(Robot.swerve);
    m_x = x;
    m_y = y;
    m_rotation = rotation;

    m_LimiterX = new SlewRateLimiter(.5);
    m_LimiterY = new SlewRateLimiter(.5);
    m_LimiterRotation = new SlewRateLimiter(.5);
  }

  @Override
  public void execute() {
    double m_xPercent = m_LimiterX.calculate(m_x.get().in(Value));
    double m_yPercent = m_LimiterY.calculate(m_y.get().in(Value));
    double m_rotationPercent = m_LimiterRotation.calculate(
      m_rotation.get().in(Value)
    );

    if (m_xPercent == 0 && m_yPercent == 0 && m_rotationPercent == 0) {
      Robot.swerve.stopMotors();
    } else {
      Robot.swerve.driveFieldRelative(
        SWERVE.MAX_SPEED.times(Constants.SWERVE.AXIS_MAX_SPEED).times(
          m_xPercent
        ),
        SWERVE.MAX_SPEED.times(Constants.SWERVE.AXIS_MAX_SPEED).times(
          m_yPercent
        ),
        SWERVE.MAX_ANGULAR_RATE.times(
          Constants.SWERVE.AXIS_MAX_ANGULAR_RATE
        ).times(m_rotationPercent)
      );
    }
  }
}
