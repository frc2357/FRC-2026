package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoMaker.Auto;
import frc.robot.commands.drive.AutoTargetLock;
import frc.robot.commands.intakepivot.AutoIntakePivotDeploy;
import frc.robot.commands.intakepivot.IntakePivotDeploy;
import frc.robot.commands.intaking.AutoIntakeUntil;
import frc.robot.commands.scoring.auto.AutoShoot;
import frc.robot.subsystems.CommandSwerveDrivetrain.AutoDriveMode;

public class LeftTrenchBump extends AutoBase {

  protected AutoRoutine m_routine;
  protected AutoTrajectory m_startTraj;

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public LeftTrenchBump() {
    super("LeftTrenchBump");
  }

  @Override
  public AutoRoutine getRoutine() {
    Auto auto = AutoMaker.newAuto(m_name);
    AutoTrajectory traj = auto.startTrajectory();

    return auto.routine();
  }
}
