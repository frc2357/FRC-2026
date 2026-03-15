package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class VelocityDrive extends Command {

  double targetMetersPerSecond;
  Dimensionless m_speedX;
  Dimensionless m_speedY;
  Dimensionless m_rotation;

  public VelocityDrive(
    Dimensionless speedX,
    Dimensionless speedY,
    Dimensionless rotation
  ) {
    addRequirements(Robot.swerve);
    SmartDashboard.putNumber("Drive Target MS", 2);
    m_speedX = speedX;
    m_speedY = speedY;
    m_rotation = rotation;
  }

  @Override
  public void initialize() {
    Robot.swerve.driveFieldRelative(
      MetersPerSecond.of(m_speedX.in(Value)),
      MetersPerSecond.of(m_speedY.in(Value)),
      RadiansPerSecond.of(m_rotation.in(Value))
    );
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
