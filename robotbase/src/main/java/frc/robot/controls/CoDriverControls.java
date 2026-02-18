package frc.robot.controls;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;
import frc.robot.commands.StopAllMotors;
import frc.robot.commands.intake.IntakeAxis;
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
    Trigger noDpad = m_controller
      .povUp()
      .negate()
      .and(m_controller.povRight().negate())
      .and(m_controller.povLeft().negate())
      .and(m_controller.povDown().negate());

    Trigger onlyLeft = m_controller
      .povUp()
      .negate()
      .and(m_controller.povRight().negate())
      .and(m_controller.povLeft())
      .and(m_controller.povDown().negate());

    Trigger onlyRight = m_controller
      .povUp()
      .negate()
      .and(m_controller.povRight())
      .and(m_controller.povLeft().negate())
      .and(m_controller.povDown().negate());

    Trigger onlyUp = m_controller
      .povUp()
      .and(m_controller.povRight().negate())
      .and(m_controller.povLeft().negate())
      .and(m_controller.povDown().negate());

    Trigger onlyDown = m_controller
      .povUp()
      .negate()
      .and(m_controller.povRight().negate())
      .and(m_controller.povLeft().negate())
      .and(m_controller.povDown());
    Trigger noLetterButtons = m_controller
      .a()
      .negate()
      .and(m_controller.b().negate())
      .and(m_controller.x().negate())
      .and(m_controller.y().negate());

    m_controller.start().onTrue(new StopAllMotors());

    // change WaitCommad to ShooterHoodAxis once it is finshed
    onlyUp.whileTrue(new WaitCommand(0));

    onlyUp
      .and(m_controller.rightTrigger())
      .whileTrue(
        Robot.shooter.axisSpeed(() ->
          Value.of(m_controller.getRightTriggerAxis())
        )
      );

    onlyUp
      .and(m_controller.leftTrigger())
      .whileTrue(
        Robot.shooter.axisSpeed(() ->
          Value.of(-m_controller.getLeftTriggerAxis())
        )
      );

    onlyLeft
      .and(m_controller.leftTrigger())
      .whileTrue(
        new IntakeAxis(() -> Value.of(-m_controller.getLeftTriggerAxis()))
      );

    onlyLeft
      .and(m_controller.rightTrigger())
      .whileTrue(
        new IntakeAxis(() -> Value.of(m_controller.getRightTriggerAxis()))
      );
  }

  private double modifyAxis(double value) {
    value = deadband(value, CONTROLLER.CODRIVER_CONTROLLER_DEADBAND);
    value = Math.copySign(
      Math.pow(value, Constants.CONTROLLER.JOYSTICK_RAMP_EXPONENT),
      value
    );
    return value;
  }

  public Dimensionless getRightX() {
    return Value.of(modifyAxis(m_controller.getRightX()));
  }

  public Dimensionless getLeftX() {
    return Value.of(modifyAxis(m_controller.getLeftX()));
  }

  public Dimensionless getRightY() {
    return Value.of(modifyAxis(-m_controller.getRightY()));
  }

  public Dimensionless getLeftY() {
    return Value.of(modifyAxis(-m_controller.getLeftY()));
  }

  public Dimensionless getRightTrigger() {
    return Value.of(-m_controller.getRightTriggerAxis());
  }

  public Dimensionless getLeftTrigger() {
    return Value.of(m_controller.getLeftTriggerAxis());
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
