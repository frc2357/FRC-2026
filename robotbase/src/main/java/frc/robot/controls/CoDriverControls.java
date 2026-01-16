package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CONTROLLER;
import frc.robot.controls.util.RumbleInterface;

public class CoDriverControls implements RumbleInterface {

  private CommandXboxController m_controller;

  public CoDriverControls() {
    m_controller = new CommandXboxController(
      CONTROLLER.CODRIVER_CONTROLLER_PORT
    );
    mapControls();
  }

  public void mapControls() {}

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
