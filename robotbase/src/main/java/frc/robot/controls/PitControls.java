package frc.robot.controls;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Constants.SHOOTER;
import frc.robot.Robot;
import frc.robot.commands.drive.TestSwerve;
import frc.robot.commands.intakepivot.IntakePivotJiggle;
import frc.robot.commands.intakerunner.IntakeRunnerAxis;
import frc.robot.commands.scoring.ScoreFeed;
import frc.robot.commands.scoring.SlowPitShoot;

public class PitControls {

  private CommandXboxController m_controller;

  public PitControls() {
    m_controller = new CommandXboxController(CONTROLLER.PIT_CONTROLLER_PORT);
    mapControls();
  }

  public void mapControls() {
    m_controller.b().whileTrue(new SlowPitShoot());

    m_controller.x().whileTrue(new ScoreFeed());

    m_controller.a().whileTrue(new TestSwerve());

    m_controller.y().whileTrue(new IntakePivotJiggle());

    m_controller
      .rightTrigger()
      .whileTrue(
        new IntakeRunnerAxis(() ->
          Percent.of(m_controller.getRightTriggerAxis())
        )
      );

    m_controller
      .povUp()
      .whileTrue(Robot.hood.setSpeed(Constants.HOOD.MANUAL_HOOD_SPEED));

    m_controller
      .povDown()
      .whileTrue(
        Robot.hood.setSpeed(Constants.HOOD.MANUAL_HOOD_SPEED.times(-1))
      );
  }
}
