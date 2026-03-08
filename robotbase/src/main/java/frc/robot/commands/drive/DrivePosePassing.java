package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.commands.scoring.SetShotTarget;
import java.util.function.Supplier;

public class DrivePosePassing extends Command {

  Supplier<Dimensionless> m_x;
  Supplier<Dimensionless> m_y;

  public DrivePosePassing(
    Supplier<Dimensionless> x,
    Supplier<Dimensionless> y
  ) {
    addRequirements(Robot.swerve);

    m_x = x;
    m_y = y;

    this.alongWith(new SetShotTarget(this::getTarget));
  }

  @Override
  public void execute() {
    Rotation2d target = Robot.scoreCalculator.getCalculatedDriveAngle();
    SmartDashboard.putNumber("Target angle", target.getDegrees());
    Robot.swerve.driveAtAngle(
      m_y.get().times(Constants.SWERVE.AXIS_MAX_SPEED).times(SWERVE.MAX_SPEED),
      m_x.get().times(Constants.SWERVE.AXIS_MAX_SPEED).times(SWERVE.MAX_SPEED),
      target
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private Translation2d getTarget() {
    Pose2d robotPose = Robot.swerve.getAllianceRelativePose2d();
    if (robotPose.getY() > FieldConstants.Hub.centerPoint.getY()) {
      return FieldConstants.Bump.Left.centerPoint;
    } else {
      return FieldConstants.Bump.Right.centerPoint;
    }
  }
}
