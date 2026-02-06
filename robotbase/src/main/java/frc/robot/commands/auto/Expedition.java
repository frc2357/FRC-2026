package frc.robot.commands.auto;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Expedition extends AutoBase {

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public Expedition() {
    super("Expedition", "Expedition");
    AutoTrajectory Expedition2 = m_routine.trajectory("Expedition");

    m_startTraj.done().onTrue(new WaitCommand(4).andThen(Expedition2.cmd()));
  }
}
