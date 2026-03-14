package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Constants.SHOOTER;
import frc.robot.Robot;
import frc.robot.commands.debug.SlowPitShoot;
import frc.robot.commands.scoring.ScoreFeed;

public class PitControls {

  private CommandXboxController m_controller;

  public PitControls() {
    m_controller = new CommandXboxController(CONTROLLER.PIT_CONTROLLER_PORT);
    mapControls();
  }

  public void mapControls() {
    m_controller.b().onTrue(new SlowPitShoot());

    m_controller.x().onTrue(new ScoreFeed());

    m_controller.a().onTrue();

    m_controller.y().onTrue();
  }
}
