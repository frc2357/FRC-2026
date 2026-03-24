package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;
import frc.robot.ShotCalculator;
import frc.robot.controls.util.RumbleInterface;

public class TuningControls implements RumbleInterface {

  private CommandXboxController m_controller;

  private int m_currentSetpointIndex = 0;

  public TuningControls() {
    m_controller = new CommandXboxController(CONTROLLER.TUNING_CONTROLLER_PORT);
    mapControls();

    updateCurveTuners();
  }

  public void mapControls() {
    m_controller
      .leftBumper()
      .onTrue(new InstantCommand(() -> cyclePrevSetpoint()));
    m_controller
      .rightBumper()
      .onTrue(new InstantCommand(() -> cycleNextSetpoint()));
  }

  private void cyclePrevSetpoint() {
    if (m_currentSetpointIndex > 0) {
      m_currentSetpointIndex--;
    } else {
      // TODO: Figure out what we want to do here (error or wrap)
    }

    updateCurveTuners();
  }

  private void cycleNextSetpoint() {
    if (m_currentSetpointIndex < ShotCalculator.SHOT_POINT_ARRAY.length - 1) {
      m_currentSetpointIndex++;
    } else {
      // TODO: Figure out what we want to do here (error or wrap)
    }

    updateCurveTuners();
  }

  private void updateCurveTuners() {
    System.out.println(
      "Updating Curve Tuners to Setpoint Index: " + m_currentSetpointIndex
    );
    Robot.shotCalculator
      .getScoringHoodCurve()
      .updateSelectedCurveIndex(
        ShotCalculator.SHOT_POINT_ARRAY[m_currentSetpointIndex]
      );
    Robot.shotCalculator
      .getScoringShooterCurve()
      .updateSelectedCurveIndex(
        ShotCalculator.SHOT_POINT_ARRAY[m_currentSetpointIndex]
      );
  }

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
