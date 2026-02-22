package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;
import frc.robot.commands.drive.DrivePoseTargetingHub;
import frc.robot.commands.drive.FlipPerspective;
import frc.robot.commands.drive.ResetPerspective;
import frc.robot.commands.intake.IntakeAxis;
import frc.robot.commands.scoring.Score;
import frc.robot.commands.scoring.VisionScore;
import frc.robot.commands.scoring.VisionTargeting;
import frc.robot.controls.util.RumbleInterface;

public class DriverControls implements RumbleInterface {

  private CommandXboxController m_controller;

  public DriverControls() {
    m_controller = new CommandXboxController(CONTROLLER.DRIVER_CONTROLLER_PORT);
    mapControls();
  }

  public void mapControls() {
    m_controller.back().onTrue(new FlipPerspective());
    m_controller.start().onTrue(new ResetPerspective());

    m_controller.leftTrigger().whileTrue(new VisionTargeting());

    m_controller
      .rightTrigger()
      .whileTrue(
        new IntakeAxis(() -> Value.of(m_controller.getRightTriggerAxis()))
      );

    m_controller.y().whileTrue(Robot.hood.setSpeed(Percent.of(10)));
    m_controller.a().whileTrue(Robot.hood.setSpeed(Percent.of(-10)));

    m_controller
      .x()
      .whileTrue(new DrivePoseTargetingHub(this::getLeftX, this::getLeftY));

    m_controller.b().whileTrue(new VisionScore(this::getLeftX, this::getLeftY));

    m_controller
      .povRight()
      .whileTrue(
        Robot.hood.setAngle(() ->
          Degrees.of(SmartDashboard.getNumber("Hood Target Degree", 1))
        )
      );

    m_controller
      .povLeft()
      .onTrue(
        new InstantCommand(() -> {
          var estimate =
            Robot.cameraManager.m_shooter.getSeedEstimateForSwerve();
          if (estimate.isPresent()) {
            Robot.swerve.setFieldRelativeTranslation2d(
              estimate.get().estimatedPose.toPose2d().getTranslation()
            );
          }
        })
      );
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
    value = deadband(value, CONTROLLER.DRIVER_CONTROLLER_DEADBAND);
    value = Math.copySign(
      Math.pow(value, Constants.CONTROLLER.JOYSTICK_RAMP_EXPONENT),
      value
    );
    return value;
  }

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
