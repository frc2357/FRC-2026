package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import java.util.function.Supplier;

public class DriveTargetingHub extends Command {

  Supplier<Dimensionless> m_x;
  Supplier<Dimensionless> m_y;
  Supplier<Dimensionless> m_rotation;

  public DriveTargetingHub(
    Supplier<Dimensionless> x,
    Supplier<Dimensionless> y
  ) {
    addRequirements(Robot.swerve);
    m_x = x;
    m_y = y;
  }

  @Override
  public void execute() {
    if (m_x.get().in(Percent) == 0 && m_y.get().in(Percent) == 0) {
      Robot.swerve.stopMotors();
    } else {
      Robot.swerve.driveAtAngle(
        m_y
          .get()
          .times(Constants.SWERVE.AXIS_MAX_SPEED)
          .times(SWERVE.MAX_SPEED),
        m_x
          .get()
          .times(Constants.SWERVE.AXIS_MAX_SPEED)
          .times(SWERVE.MAX_SPEED),
        Rotation2d.kZero
      );
    }
  }
}
