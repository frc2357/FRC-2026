package frc.robot.commands.scoring.teleop;

import frc.robot.Constants;
import frc.robot.commands.scoring.Score;

public class HubScore extends Score {

  public HubScore() {
    super(
      Constants.SHOOTER.SETPOINTS.HUB_SHOT,
      Constants.HOOD.SETPOINTS.HUB_SHOT
    );
  }
}
