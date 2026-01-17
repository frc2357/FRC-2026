package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class PullYaw extends Command {

  @Override
  public void initialize() {
    Robot.backLeftCam.setPipeline(0);
  }

  @Override
  public void execute() {
    double yaw = Robot.backLeftCam.getTargetYaw(16, 50);
    System.out.println(yaw);
    SmartDashboard.putNumber("camera yaw", yaw);
  }
}
