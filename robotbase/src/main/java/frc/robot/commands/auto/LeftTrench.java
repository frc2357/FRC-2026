package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoMaker.Auto;
import frc.robot.commands.drive.AutoTargetLock;
import frc.robot.commands.intakepivot.AutoIntakePivotDeploy;
import frc.robot.commands.intakepivot.IntakePivotJiggle;
import frc.robot.commands.intakerunner.IntakeRunnerUntil;
import frc.robot.commands.scoring.auto.AutoScore;
import frc.robot.commands.scoring.auto.AutoShoot;

public class LeftTrench extends AutoBase {

  protected AutoRoutine m_routine;
  protected AutoTrajectory m_startTraj;

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public LeftTrench() {
    super("LeftTrench");
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
      .onTrue(Robot.shooter.autoSetVelocity(RotationsPerSecond.of(55)));
    // traj
    //   .done()
    //   .onTrue(
    //     new AutoScore(RotationsPerSecond.of(55), Degrees.of(13)).alongWith(
    //       new IntakePivotJiggle()
    //     )
    //   );
    traj.done().onTrue(new AutoShoot());
    traj.done().onTrue(new AutoTargetLock());

    return auto.routine();
  }
}
