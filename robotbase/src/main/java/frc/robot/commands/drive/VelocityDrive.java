package frc.robot.commands.drive;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class VelocityDrive extends Command {

  LinearVelocity m_x;
  LinearVelocity m_y;
  AngularVelocity m_rotation;

  public VelocityDrive(
    LinearVelocity x,
    LinearVelocity y,
    AngularVelocity rotation
  ) {
    addRequirements(Robot.swerve);
    SmartDashboard.putNumber("Drive Target MS", 2);
    m_x = x;
    m_y = y;
    m_rotation = rotation;
  }

  @Override
  public void initialize() {
    Robot.swerve.driveFieldRelative(m_x, m_y, m_rotation);
  }

  @Override
  public void execute() {
    double RotationsPerSecond = Robot.swerve
      .getModule(0)
      .getDriveMotor()
      .getVelocity()
      .getValue()
      .in(Units.RotationsPerSecond);
    SmartDashboard.putNumber("Drive RPS", RotationsPerSecond);
    SmartDashboard.putNumber(
      "Calculated Drive MS",
      Robot.swerve.getModule(0).getCurrentState().speedMetersPerSecond
    );
  }
}
