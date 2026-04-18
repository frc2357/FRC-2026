package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoMaker.Auto;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;

public class SeedTest extends AutoBase {

  protected AutoRoutine m_routine;
  protected AutoTrajectory m_startTraj;

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public SeedTest() {
    super("SeedTest");
  }

  @Override
  public AutoRoutine getRoutine() {
    Auto auto = AutoMaker.newAuto(m_name);
    var startTraj = auto.startTrajectory();
    var initialPose = startTraj.getInitialPose().get();

    Robot.cameraManager.m_limelightShooter.seedGyroMethod(
      new Orientation3d(
        new Rotation3d(initialPose.getRotation()),
        new AngularVelocity3d(
          DegreesPerSecond.zero(),
          DegreesPerSecond.zero(),
          DegreesPerSecond.zero()
        )
      )
    );
    Robot.swerve.setFieldRelativePose2d(initialPose);
    Robot.cameraManager.m_limelightShooter.useInternalImu();

    return auto.routine();
  }
}
