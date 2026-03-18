package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoMaker.Auto;
import frc.robot.commands.intakepivot.AutoIntakePivotDeploy;
import frc.robot.commands.intakepivot.IntakePivotJiggle;
import frc.robot.commands.intakerunner.IntakeRunnerUntil;
import frc.robot.commands.scoring.auto.AutoScore;

public class RightTrench extends AutoBase {

  protected AutoRoutine m_routine;
  protected AutoTrajectory m_startTraj;

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public RightTrench() {
    super("RightTrench");
  }

  @Override
  public AutoRoutine getRoutine() {
    Auto auto = AutoMaker.newAuto(m_name);
    AutoTrajectory traj = auto.startTrajectory();
    traj.active().onTrue(new AutoIntakePivotDeploy());
    traj
      .atTime("StartIntake")
      .onTrue(new IntakeRunnerUntil(traj.atTime("StopIntake")));
    traj
      .atTime("StopIntake")
      .onTrue(Robot.shooter.autoSetVelocity(RotationsPerSecond.of(56)));
    traj
      .done()
      .onTrue(
        new AutoScore(RotationsPerSecond.of(56), Degrees.of(13)).alongWith(
          new IntakePivotJiggle()
        )
      );

    return auto.routine();
  }
}
