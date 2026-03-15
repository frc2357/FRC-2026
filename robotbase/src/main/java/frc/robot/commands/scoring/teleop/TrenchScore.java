package frc.robot.commands.scoring.teleop;

import frc.robot.Constants;
import frc.robot.commands.scoring.Score;

public class TrenchScore extends Score {

  public TrenchScore() {
    super(
      Constants.SHOOTER.SETPOINTS.TRENCH_SHOT,
      Constants.HOOD.SETPOINTS.TRENCH_SHOT
    );
  }
}
