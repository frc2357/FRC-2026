package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.commands.auto.AutoMaker.Auto;
import frc.robot.commands.drive.AutoTargetLock;
import frc.robot.commands.intakepivot.AutoIntakePivotDeploy;
import frc.robot.commands.intaking.AutoIntakeUntil;
import frc.robot.commands.scoring.auto.AutoShoot;
import frc.robot.subsystems.CommandSwerveDrivetrain.AutoDriveMode;

public class DepotSingleSwipeTrench extends AutoBase {

  protected AutoRoutine m_routine;
  protected AutoTrajectory m_startTraj;

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public DepotSingleSwipeTrench() {
    super("DepotSingleSwipeTrench");
  }

  @Override
  public AutoRoutine getRoutine() {
    Auto auto = AutoMaker.newAuto(m_name);
    AutoTrajectory traj = auto.startTrajectory();

    traj.active().onTrue(new AutoIntakePivotDeploy());

    traj
      .atTime("StartIntake")
      .onTrue(new AutoIntakeUntil(traj.atTime("StopIntake")));

    // SOTF after depot
    traj
      .atTime("StartShootDepot")
      .onTrue(new AutoShoot().until(traj.atTime("StopShootDepot")));
    traj
      .atTime("StartShootDepot")
      .onTrue(
        new SetAutoDriveMode(AutoDriveMode.TARGET_LOCK).until(
          traj.atTime("StopShootDepot")
        )
      );

    traj
      .atTime("StartIntake2")
      .onTrue(new AutoIntakeUntil(traj.atTime("StopIntake2")));

    traj.done().onTrue(new AutoShoot());
    traj.done().onTrue(new AutoTargetLock());

    return auto.routine();
  }
}
