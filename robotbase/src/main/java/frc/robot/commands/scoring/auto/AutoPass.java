package frc.robot.commands.scoring.auto;

import frc.robot.Constants.AUTO;

public class AutoPass extends AutoScore {

  public AutoPass() {
    super(AUTO.AUTO_PASSING_SHOOTER_VELOCITY, AUTO.AUTO_PASSING_HOOD_ANGLE);
  }
}
