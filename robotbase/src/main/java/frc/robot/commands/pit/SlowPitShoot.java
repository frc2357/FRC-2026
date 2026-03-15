package frc.robot.commands.pit;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.scoring.Score;

public class SlowPitShoot extends ParallelCommandGroup {

  public SlowPitShoot() {
    super(
      new Score((Constants.SHOOTER.SLOW_SPEED), (Constants.HOOD.SETPOINTS.HOME))
    );
  }
}
