// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodTuningSubsystem;
import frc.robot.subsystems.ShooterTuningSubsystem;

public class Robot extends TimedRobot {

  Command m_command;

  XboxController m_controller;

  AddressableLED m_LED;
  AddressableLEDBuffer m_ledBuffer;
  LEDPattern m_rainbow = LEDPattern.rainbow(255, 50);

  LEDPattern m_scrollingRainbow;
  public boolean camera = true;

<<<<<<< HEAD
  ShooterTuningSubsystem m_shooter;
  HoodTuningSubsystem m_hood;

=======
>>>>>>> 7ad775b (Added Rainbow and orange so far.)
  public Robot() {
    //m_shooter = new ShooterTuningSubsystem();
    m_hood = new HoodTuningSubsystem();
    m_controller = new XboxController(0);
<<<<<<< HEAD
=======
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_LED = new AddressableLED(7);
    Distance kLedSpacing = Meters.of(1 / 64.0);

    m_ledBuffer = new AddressableLEDBuffer(1);

    m_LED.setLength(m_ledBuffer.getLength());
    m_LED.start();

    m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(
      MetersPerSecond.of(0.5),
      kLedSpacing
    );
>>>>>>> 7ad775b (Added Rainbow and orange so far.)
  }

  @Override
  public void robotPeriodic() {
    //m_shooter.updateDashboard();
    m_hood.updateDashboard();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    //m_shooter.teleopPeriodic();
    m_hood.teleopPeriodic();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
    //m_shooter.setAxisSpeed(Value.of(-m_controller.getRightY()));
    m_hood.setAxisSpeed(Value.of(-m_controller.getRightY()));
  }
}
