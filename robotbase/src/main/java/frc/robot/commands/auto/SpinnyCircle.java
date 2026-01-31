package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class SpinnyCircle extends AutoBase {

  protected AutoRoutine m_routine;
  protected AutoTrajectory m_startTraj;

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public SpinnyCircle() {
    super("SpinnyCircle");
  }
}
