package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;

public class ResetOdometry extends Command {

  Pose2d m_initialPose;

  public ResetOdometry(Pose2d initialPose) {
    m_initialPose = initialPose;
  }

  @Override
  public void initialize() {
    Robot.cameraManager.m_limelightShooter.seedGyroMethod(
      new Orientation3d(
        new Rotation3d(m_initialPose.getRotation()),
        new AngularVelocity3d(
          DegreesPerSecond.zero(),
          DegreesPerSecond.zero(),
          DegreesPerSecond.zero()
        )
      )
    );
    Robot.swerve.setFieldRelativePose2d(m_initialPose);
    Robot.cameraManager.m_limelightShooter.useInternalImu();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
