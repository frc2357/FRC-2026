package frc.robot.commands.scoring.teleop;

import frc.robot.Constants;
import frc.robot.commands.scoring.Score;

public class OutpostScore extends Score {

  public OutpostScore() {
    super(
      Constants.SHOOTER.SETPOINTS.OUTPOST_SHOT,
      Constants.HOOD.SETPOINTS.OUTPOST_SHOT
    );
  }
}
