package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import java.util.function.Supplier;

public class DriveTargetingHub extends Command {

  Supplier<Dimensionless> m_x;
  Supplier<Dimensionless> m_y;

  public DriveTargetingHub(
    Supplier<Dimensionless> x,
    Supplier<Dimensionless> y
  ) {
    addRequirements(Robot.swerve);
    m_x = x;
    m_y = y;

    SmartDashboard.putNumber("Target Rotation", 0);
  }

  @Override
  public void initialize() {
    Robot.backLeftCam.setPipeline(0); // April tag pipeline TODO: make a constant
  }

  @Override
  public void execute() {
    Robot.swerve.driveAtAngle(
      m_y.get().times(Constants.SWERVE.AXIS_MAX_SPEED).times(SWERVE.MAX_SPEED),
      m_x.get().times(Constants.SWERVE.AXIS_MAX_SPEED).times(SWERVE.MAX_SPEED),
      Rotation2d.fromDegrees(SmartDashboard.getNumber("Target Rotation", 0))
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
