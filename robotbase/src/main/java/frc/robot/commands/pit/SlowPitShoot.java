package frc.robot.commands.pit;

import frc.robot.Constants;
import frc.robot.commands.scoring.auto.AutoScore;

public class SlowPitShoot extends AutoScore {

  public SlowPitShoot() {
    super(
      () -> Constants.SHOOTER.SETPOINTS.PIT_SHOT,
      () -> Constants.HOOD.SETPOINTS.PIT_SHOT
    );
  }
}
