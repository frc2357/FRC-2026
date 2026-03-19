package frc.robot.commands.pit;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.scoring.PitScore;

public class SlowPitShoot extends ParallelCommandGroup {

  public SlowPitShoot() {
    super(
      new PitScore(
        Constants.SHOOTER.SETPOINTS.PIT_SHOT,
        Constants.HOOD.SETPOINTS.PIT_SHOT
      )
    );
  }
}
