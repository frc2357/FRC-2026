package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Value;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.SWERVE;

public class DefaultDrive extends Command {
    Supplier<Dimensionless> m_xSupplier;
    Supplier<Dimensionless> m_ySupplier;
    Supplier<Dimensionless> m_rotationSupplier;

    SlewRateLimiter m_xLimiter;
    SlewRateLimiter m_yLimiter;

    public DefaultDrive(
            Supplier<Dimensionless> xSupplier,
            Supplier<Dimensionless> ySupplier,
            Supplier<Dimensionless> rotationSupplier) {
        addRequirements(Robot.swerve);
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_rotationSupplier = rotationSupplier;

        m_xLimiter = new SlewRateLimiter(SWERVE.SLEW_RATE_LIMIT);
        m_yLimiter = new SlewRateLimiter(SWERVE.SLEW_RATE_LIMIT);
    }

    @Override
    public void execute() {
        double x = m_xLimiter.calculate(m_xSupplier.get().in(Value));
        double y = m_yLimiter.calculate(m_ySupplier.get().in(Value));
        double rotation = m_rotationSupplier.get().in(Value);

        if (x == 0 && y == 0 && rotation == 0) {
            Robot.swerve.stopMotors();
        } else {
            Robot.swerve.driveFieldRelative(
                    SWERVE.MAX_SPEED.times(y),
                    SWERVE.MAX_SPEED.times(x),
                    SWERVE.MAX_ANGULAR_RATE.times(rotation));
        }
    }
}
