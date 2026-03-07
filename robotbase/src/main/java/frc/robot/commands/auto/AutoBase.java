package frc.robot.commands.auto;

import static frc.robot.Constants.CHOREO.DEFAULT_AUTO_FACTORY;
import static frc.robot.Constants.CHOREO.TARGET_LOCK_AUTO_FACTORY;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.util.VariableWaitCommand;

public class AutoBase {

  // TODO: Rewrite this (possibly from scratch) to join autos according to our plans for this year

  protected final AutoRoutine m_defaultRoutine;
  protected final AutoRoutine m_targetLockRoutine;
  private final String m_name;

  /**
   * Initialize an auto routine pair.
   * Default and TargetLock routines are always created; child classes decide which trajectories to use.
   * @param name Name of the auto routine
   */
  public AutoBase(String name) {
    m_name = name;
    m_defaultRoutine = DEFAULT_AUTO_FACTORY.newRoutine(m_name + " - (Default)");
    m_targetLockRoutine = TARGET_LOCK_AUTO_FACTORY.newRoutine(
      m_name + " - (Target Lock)"
    );
  }

  /**
   * Create a trajectory from the default routine.
   */
  protected AutoTrajectory defaultTrajectory(String trajectoryName) {
    return m_defaultRoutine.trajectory(trajectoryName);
  }

  /**
   * Create a trajectory from the target-lock routine.
   */
  protected AutoTrajectory targetLockTrajectory(String trajectoryName) {
    return m_targetLockRoutine.trajectory(trajectoryName);
  }

  protected AutoTrajectory setStartTrajectory(AutoTrajectory startTraj) {
    m_defaultRoutine.observe(m_targetLockRoutine.active());
    m_defaultRoutine
      .active()
      .and(m_targetLockRoutine.active())
      .onTrue(
        Commands.sequence(
          new VariableWaitCommand(() ->
            SmartDashboard.getNumber("wait seconds", 0)
          ),
          startTraj.resetOdometry(),
          startTraj.cmd()
        )
      );

    return startTraj;
  }

  public AutoRoutine getDefaultRoutine() {
    return m_defaultRoutine;
  }

  public AutoRoutine getTargetLockRoutine() {
    return m_targetLockRoutine;
  }

  @Override
  public String toString() {
    return m_name;
  }
}
