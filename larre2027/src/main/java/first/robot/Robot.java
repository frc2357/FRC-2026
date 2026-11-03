// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.framework.OpModeRobot;

import first.robot.mechanisms.flywheel.Flywheel;

public class Robot extends OpModeRobot {

  public static Flywheel flywheel;
  public Robot() {
    flywheel = new Flywheel();
  }


  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
  }
}
