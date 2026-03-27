package frc.robot.commands.scoring.teleop;

import frc.robot.Constants;
import frc.robot.commands.scoring.auto.AutoScore;

// Extending AutoScore to avoid all the constraints of TeleopScore
public class OutpostScore extends AutoScore {

  public OutpostScore() {
    super(
      Constants.SHOOTER.SETPOINTS.OUTPOST_SHOT,
      Constants.HOOD.SETPOINTS.OUTPOST_SHOT
    );
  }
}
