// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.controls.CoDriverControls;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private DriverControls driverControls;
  private CoDriverControls coDriverControls;

  private static Command m_defaultDrive;

  public static CommandSwerveDrivetrain swerve;

  private final Telemetry logger = new Telemetry(
      Constants.SWERVE.MAX_SPEED.in(Units.MetersPerSecond));

  public Robot() {
    swerve = TunerConstants.createDrivetrain();

    driverControls = new DriverControls();
    coDriverControls = new CoDriverControls();
    m_defaultDrive = new DefaultDrive(
        driverControls::getLeftY,
        driverControls::getLeftX,
        driverControls::getRightX);

    swerve.registerTelemetry(logger::telemeterize);
    swerve.setDefaultCommand(m_defaultDrive);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
