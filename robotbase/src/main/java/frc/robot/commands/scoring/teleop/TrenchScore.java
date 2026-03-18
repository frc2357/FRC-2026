package frc.robot.commands.scoring.teleop;

import frc.robot.Constants;
import frc.robot.commands.scoring.auto.AutoScore;

// Extending AutoScore to avoid all the constraints of TeleopScore
public class TrenchScore extends AutoScore {

  public TrenchScore() {
    super(
      Constants.SHOOTER.SETPOINTS.TRENCH_SHOT,
      Constants.HOOD.SETPOINTS.TRENCH_SHOT
    );
  }
}
