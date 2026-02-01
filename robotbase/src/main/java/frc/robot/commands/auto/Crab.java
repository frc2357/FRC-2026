package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Percent;

import frc.robot.commands.intake.IntakeSetSpeed;

public class Crab extends AutoBase {

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public Crab() {
    super("Crab");
    m_startTraj.active().onTrue(new IntakeSetSpeed(Percent.of(-70)));
  }
}
