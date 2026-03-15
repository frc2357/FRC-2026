package frc.robot.commands.auto;

import static frc.robot.Constants.CHOREO.AUTO_FACTORY;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.util.VariableWaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain.AutoDriveMode;

public final class AutoMaker {

  public record Auto(AutoRoutine routine, AutoTrajectory startTrajectory) {}

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public static Auto newAuto(String name) {
    return AutoMaker.newAuto(name, name);
  }

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   * @param startTraj Trajectory to start with
   */
  public static Auto newAuto(String name, String startTrajName) {
    // This is how you make an auto routine. The name should be essentially the same as the name of the function it is in.
    AutoRoutine routine = AUTO_FACTORY.newRoutine(name);

    // This is how you make a trajectory to put into the AutoRoutine. The trajectoryName is the name of the file in Choreo.
    // Any deviation from that name will result in the file not being found.
    AutoTrajectory startTraj = routine.trajectory(startTrajName);
    // This is how you reset the odometry and make the routine use a trajectory. This is a veyr regular thing that you will have to do.
    routine
      // .active() returns a trigger that is true while the AutoRoutine is running
      .active()
      // .onTrue() lets us run commands once that trigger becomes true.
      .onTrue(
        // Commands.sequence() lets us sequence commands to run, letting us run multiple commands per trigger.
        Commands.sequence(
          new VariableWaitCommand(() ->
            SmartDashboard.getNumber("wait seconds", 0)
          ),
          // This command resets the odometry, and it MUST be run on the starting traj, or very bad things will happen.
          startTraj.resetOdometry(),
          // This runs the trajectory that was loaded earlier. This is needed to make the AutoRoutine actually run the trajectory, instead of doing nothing.
          startTraj.cmd()
        )
      );

    return new Auto(routine, startTraj);
  }

  /**
   * Returns a routine that sets the odomoetry and follows a single path
   * @param Name of the path
   * @return The auto routine to follow the path
   */
  public static AutoRoutine simpleAutoRoutine(String name) {
    return AutoMaker.newAuto(name).routine;
  }

  public static AutoTrajectory newTrajectory(
    AutoRoutine routine,
    String trajName,
    AutoDriveMode driveMode
  ) {
    AutoTrajectory traj = routine.trajectory(trajName);
    traj.active().whileTrue(new SetAutoDriveMode(driveMode));
    return traj;
  }
}
