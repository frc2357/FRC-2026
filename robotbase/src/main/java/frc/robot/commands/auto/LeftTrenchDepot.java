package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.commands.auto.AutoMaker.Auto;
import frc.robot.commands.intakepivot.AutoIntakePivotDeploy;
import frc.robot.commands.intakerunner.IntakeRunnerUntil;
import frc.robot.commands.scoring.auto.AutoScore;

public class LeftTrenchDepot extends AutoBase {

  protected AutoRoutine m_routine;
  protected AutoTrajectory m_startTraj;

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public LeftTrenchDepot() {
    super("LeftTrenchDepot");
  }

  @Override
  public AutoRoutine getRoutine() {
    Auto auto = AutoMaker.newAuto(m_name);
    AutoTrajectory traj = auto.startTrajectory();
    traj.active().onTrue(new AutoIntakePivotDeploy());
    traj
      .atTime("StartIntake1")
      .onTrue(new IntakeRunnerUntil(traj.atTime("StopIntake1")));
    traj.atTime("StartIntake2").onTrue(new IntakeRunnerUntil(traj.done()));
    traj
      .atTime("StartShooter")
      .onTrue(new AutoScore(RotationsPerSecond.of(60), Degrees.of(16)));

    return auto.routine();
  }
}
