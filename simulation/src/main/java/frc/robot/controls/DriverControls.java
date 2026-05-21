package frc.robot.controls;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.controls.util.RumbleInterface;

public class DriverControls implements RumbleInterface {

  private CommandXboxController m_controller;

  public DriverControls() {
    m_controller = new CommandXboxController(
        Constants.CONTROLLER.DRIVER_CONTROLLER_PORT);
    mapControls();
  }

  private void mapControls() {
  }

  public Dimensionless getRightX() {
    return Value.of(modifyAxis(-m_controller.getRightX()));
  }

  public Dimensionless getLeftX() {
    return Value.of(modifyAxis(-m_controller.getLeftX()));
  }

  public Dimensionless getLeftY() {
    return Value.of(modifyAxis(-m_controller.getLeftY()));
  }

  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private double modifyAxis(double value) {
    value = deadband(value, Constants.CONTROLLER.DRIVER_CONTROLLER_DEADBAND);
    value = Math.copySign(Math.pow(value, Constants.CONTROLLER.JOYSTICK_RAMP_EXPONENT), value);
    return value;
  }

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
