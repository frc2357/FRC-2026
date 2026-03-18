package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.commands.auto.AutoMaker.Auto;
import frc.robot.subsystems.CommandSwerveDrivetrain.AutoDriveMode;

public class TargetLockTest extends AutoBase {

  protected AutoRoutine m_routine;
  protected AutoTrajectory m_startTraj;

  public TargetLockTest() {
    super("TargetLockTest");
  }

  @Override
  public AutoRoutine getRoutine() {
    Auto auto = AutoMaker.newAuto("FlipTest");
    AutoTrajectory traj = auto.startTrajectory();
    traj.active().whileTrue(new SetAutoDriveMode(AutoDriveMode.TARGET_LOCK));
    return auto.routine();
  }
}
