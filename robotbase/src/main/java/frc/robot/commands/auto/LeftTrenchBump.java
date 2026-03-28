package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoMaker.Auto;
import frc.robot.commands.drive.AutoTargetLock;
import frc.robot.commands.intakepivot.IntakePivotDeploy;
import frc.robot.commands.intakerunner.IntakeRunnerUntil;
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

    // First pass into neutral zone
    traj
      .atTime("StartIntake")
      .onTrue(new IntakeRunnerUntil(traj.atTime("StopIntake")));
    traj
      .atTime("StopIntake")
      .onTrue(Robot.shooter.autoSetVelocity(Constants.AUTO.AUTO_SHOOTER_IDLE));

    // Shoot and move from bump to trench
    traj
      .atTime("StartShoot")
      .onTrue(new AutoShoot().until(traj.atTime("EndShoot")));
    traj
      .atTime("StartShoot")
      .onTrue(new SetAutoDriveMode(AutoDriveMode.TARGET_LOCK));

    // Prepare for second swipe
    traj.atTime("EndShoot").onTrue(new SetAutoDriveMode(AutoDriveMode.DEFAULT));
    traj.atTime("EndShoot").onTrue(new IntakePivotDeploy());

    // Intake on second swipe
    traj
      .atTime("StartIntake2")
      .onTrue(new IntakeRunnerUntil(traj.atTime("StopIntake2")));
    traj
      .atTime("StopIntake2")
      .onTrue(Robot.shooter.autoSetVelocity(Constants.AUTO.AUTO_SHOOTER_IDLE));

    // End with shooting by the bump
    traj.done().onTrue(new AutoShoot());
    traj.done().onTrue(new AutoTargetLock());

    return auto.routine();
  }
}
