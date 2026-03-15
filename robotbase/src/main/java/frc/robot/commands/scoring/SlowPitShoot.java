package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;

public class SlowPitShoot extends ParallelCommandGroup {

  public SlowPitShoot() {
    super(
      new Score((Constants.SHOOTER.SLOW_SPEED), (Constants.HOOD.SETPOINTS.HOME))
    );
  }
}
