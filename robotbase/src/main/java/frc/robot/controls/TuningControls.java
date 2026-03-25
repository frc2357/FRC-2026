package frc.robot.controls;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Constants.HOOD;
import frc.robot.Constants.SHOOTER;
import frc.robot.Robot;
import frc.robot.ShooterCurveManager;
import frc.robot.controls.util.RumbleInterface;
import frc.robot.networkTables.CurveTuner;

public class TuningControls implements RumbleInterface {

  private CommandXboxController m_controller;

  private int m_currentSetpointIndex = 0;

  public TuningControls() {
    m_controller = new CommandXboxController(CONTROLLER.TUNING_CONTROLLER_PORT);
    mapControls();
    updateSmartDashboard();
  }

  public void mapControls() {
    m_controller
      .leftBumper()
      .onTrue(new InstantCommand(() -> cyclePrevSetpoint()));
    m_controller
      .rightBumper()
      .onTrue(new InstantCommand(() -> cycleNextSetpoint()));

    m_controller
      .y()
      .onTrue(
        createTuneCurveValueCommand(
          Robot.shooterCurveManager.getScoringHoodCurve(),
          HOOD.TUNING_STEP.magnitude()
        )
      );
    m_controller
      .a()
      .onTrue(
        createTuneCurveValueCommand(
          Robot.shooterCurveManager.getScoringHoodCurve(),
          -HOOD.TUNING_STEP.magnitude()
        )
      );
    m_controller
      .povUp()
      .onTrue(
        createTuneCurveValueCommand(
          Robot.shooterCurveManager.getScoringShooterCurve(),
          SHOOTER.TUNING_STEP.magnitude()
        )
      );
    m_controller
      .povDown()
      .onTrue(
        createTuneCurveValueCommand(
          Robot.shooterCurveManager.getScoringShooterCurve(),
          -SHOOTER.TUNING_STEP.magnitude()
        )
      );
  }

  private void cyclePrevSetpoint() {
    if (m_currentSetpointIndex > 0) {
      m_currentSetpointIndex--;
    }
    updateSmartDashboard();
  }

  private void cycleNextSetpoint() {
    if (
      m_currentSetpointIndex < ShooterCurveManager.SHOT_POINT_ARRAY.length - 1
    ) {
      m_currentSetpointIndex++;
    }
    updateSmartDashboard();
  }

  private void updateSmartDashboard() {
    Distance currentDistance = ShooterCurveManager
      .SHOT_POINT_ARRAY[m_currentSetpointIndex];
    SmartDashboard.putString(
      "Current Tuning Setpoint",
      String.format(
        "Setpoint %d: %.2f %s",
        m_currentSetpointIndex + 1,
        currentDistance.magnitude(),
        currentDistance.unit().symbol()
      )
    );
  }

  private Command createTuneCurveValueCommand(
    CurveTuner<?, ?> tuner,
    double delta
  ) {
    return new InstantCommand(() -> {
      String key = getPreferencesKey(tuner);

      double oldVal = Preferences.getDouble(key, 0);
      double newVal = oldVal + delta;

      Preferences.setDouble(key, newVal);
    });
  }

  private String getPreferencesKey(CurveTuner<?, ?> tuner) {
    return String.format(
      "%s/Setpoint %d: %.2f %s",
      tuner.getName(),
      m_currentSetpointIndex + 1,
      ShooterCurveManager.SHOT_POINT_ARRAY[m_currentSetpointIndex].magnitude(),
      ShooterCurveManager
        .SHOT_POINT_ARRAY[m_currentSetpointIndex].unit().symbol()
    );
  }

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
