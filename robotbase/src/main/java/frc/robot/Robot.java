// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.StopAllMotors;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.spindexer.SpindexerAxis;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.PhotonVisionCamera;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Spindexer;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private static DriverControls m_driverControls;
  private static Command m_defaultDrive;

  public static CommandSwerveDrivetrain swerve;

  public static PhotonVisionCamera backLeftCam;
  public static Spindexer spindexer;
  public static Intake intake;
  public static Shooter shooter;
  public static Hood hood;
  public static Outtake outtake;

  private final Telemetry logger = new Telemetry(
    Constants.SWERVE.MAX_SPEED.in(Units.MetersPerSecond)
  );
  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
    .withTimestampReplay()
    .withJoystickReplay();

  public Robot() {
    swerve = TunerConstants.createDrivetrain();

    spindexer = new Spindexer();
    intake = new Intake();
    shooter = new Shooter();
    hood = new Hood();
    outtake = new Outtake();

    // backLeftCam = new PhotonVisionCamera(
    //   Constants.PHOTON_VISION.BACK_LEFT_CAM.NAME,
    //   Constants.PHOTON_VISION.BACK_LEFT_CAM.ROBOT_TO_CAM_TRANSFORM
    // );

    swerve.registerTelemetry(logger::telemeterize);

    m_driverControls = new DriverControls();
    m_defaultDrive = new DefaultDrive(
      m_driverControls::getLeftX,
      m_driverControls::getLeftY,
      m_driverControls::getRightX
    );
    spindexer = new Spindexer();
    spindexer = new Spindexer();

    backLeftCam = new PhotonVisionCamera(
      Constants.PHOTON_VISION.BACK_LEFT_CAM.NAME,
      Constants.PHOTON_VISION.BACK_LEFT_CAM.ROBOT_TO_CAM_TRANSFORM
    );

    swerve.registerTelemetry(logger::telemeterize);
    Robot.swerve.setDefaultCommand(m_defaultDrive);
    SmartDashboard.putNumber("Spindexer", 0.0);
    Robot.spindexer.setDefaultCommand(
      new SpindexerAxis(() -> {
        return Value.of(SmartDashboard.getNumber("Spindexer", 0.0));
      })
    );
  }

  @Override
  public void robotPeriodic() {
    // backLeftCam.updateResult();
    m_timeAndJoystickReplay.update();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().schedule(new StopAllMotors());
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
