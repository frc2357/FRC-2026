package frc.robot.controls;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;
import frc.robot.commands.drive.CrossWheels;
import frc.robot.commands.intakepivot.IntakePivotJiggle;
import frc.robot.commands.intakerunner.IntakeRunnerAxis;
import frc.robot.commands.pit.CleanFeed;
import frc.robot.commands.pit.SlowPitShoot;
import frc.robot.commands.pit.TestSwerve;

public class PitControls {

  private CommandXboxController m_controller;

  public PitControls() {
    m_controller = new CommandXboxController(CONTROLLER.PIT_CONTROLLER_PORT);
    mapControls();
  }

  public void mapControls() {
    m_controller.b().whileTrue(new SlowPitShoot());

    m_controller.x().onTrue(new CrossWheels());

    m_controller.a().whileTrue(new TestSwerve());

    m_controller.y().whileTrue(new IntakePivotJiggle());

    m_controller
      .rightBumper()
      .whileTrue(
        new IntakeRunnerAxis(() -> Constants.INTAKE_RUNNER.CLEAN_SPEED)
      );

    m_controller.leftBumper().whileTrue(new CleanFeed());

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
