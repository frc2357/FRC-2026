package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class VelocityDrive extends Command {

  double targetMetersPerSecond;

  public VelocityDrive() {
    addRequirements(Robot.swerve);
    SmartDashboard.putNumber("Drive Target MS", 2);
  }

  @Override
  public void initialize() {
    Robot.swerve.driveFieldRelative(
      MetersPerSecond.of(SmartDashboard.getNumber("Drive Target MS", 2)),
      MetersPerSecond.zero(),
      RadiansPerSecond.zero()
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
  }
}
