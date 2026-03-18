package frc.robot.commands.scoring.teleop;

import frc.robot.Constants;
import frc.robot.commands.scoring.auto.AutoScore;

// Extending AutoScore to avoid all the constraints of TeleopScore
public class HubScore extends AutoScore {

  public HubScore() {
    super(
      Constants.SHOOTER.SETPOINTS.HUB_SHOT,
      Constants.HOOD.SETPOINTS.HUB_SHOT
    );
  }
}
