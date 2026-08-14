package frc.robot.commands.shooter;

import frc.robot.Constants.SHOOTER;

public class ShooterSetIdleVelocity extends ShooterSetVelocity {

  public ShooterSetIdleVelocity() {
    super(SHOOTER.SETPOINTS.IDLE_SPEED);
  }
}
