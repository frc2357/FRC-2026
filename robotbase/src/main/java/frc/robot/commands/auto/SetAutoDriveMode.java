package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain.AutoDriveMode;

public class SetAutoDriveMode extends Command {

  private AutoDriveMode m_previousAutoDriveMode;
  private AutoDriveMode m_newAutoDriveMode;

  public SetAutoDriveMode(AutoDriveMode newAutoDriveMode) {
    m_newAutoDriveMode = newAutoDriveMode;
  }

  @Override
  public void initialize() {
    m_previousAutoDriveMode = Robot.swerve.getAutoDriveMode();
    Robot.swerve.setAutoDriveMode(m_newAutoDriveMode);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.setAutoDriveMode(m_previousAutoDriveMode);
  }
}
