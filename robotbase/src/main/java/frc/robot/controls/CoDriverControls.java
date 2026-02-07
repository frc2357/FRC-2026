package frc.robot.controls;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.commands.StopAllMotors;
import frc.robot.commands.StopAllMotors;
import frc.robot.commands.intake.IntakeAxis;
import frc.robot.commands.intake.IntakeSetSpeed;
import frc.robot.commands.shooter.ShooterAxis;
import frc.robot.commands.spindexer.SpindexerAxis;
import frc.robot.controls.util.RumbleInterface;

public class CoDriverControls implements RumbleInterface {

  private CommandXboxController m_controller;

  public CoDriverControls() {
    m_controller = new CommandXboxController(
      CONTROLLER.CODRIVER_CONTROLLER_PORT
    );
    mapControls();
  }

  public void mapControls() {
    m_controller.povUp().whileTrue(new ShooterAxis(this::getRightY));

    m_controller.povUpLeft().whileTrue(new IntakeAxis(this::getRightY));

    m_controller
      .leftTrigger()
      .whileTrue(new SpindexerAxis(this::getLeftTrigger));

    m_controller.start().onTrue(new StopAllMotors());
  }

  private double modifyAxis(double value) {
    value = deadband(value, CONTROLLER.DRIVER_CONTROLLER_DEADBAND);
    value = Math.copySign(
      Math.pow(value, Constants.CONTROLLER.JOYSTICK_RAMP_EXPONENT),
      value
    );
    return value;
  }

  public Dimensionless getRightX() {
    return Value.of(modifyAxis(-m_controller.getRightX()));
  }

  public Dimensionless getLeftX() {
    return Value.of(modifyAxis(-m_controller.getLeftX()));
  }

  public Dimensionless getRightY() {
    return Value.of(modifyAxis(-m_controller.getRightY()));
  }

  public Dimensionless getLeftY() {
    return Value.of(modifyAxis(-m_controller.getLeftY()));
  }

  public Dimensionless getRightTrigger() {
    return Value.of(modifyAxis(-m_controller.getRightTriggerAxis()));
  }

  public Dimensionless getLeftTrigger() {
    return Value.of(modifyAxis(-m_controller.getLeftTriggerAxis()));
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

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
