package frc.robot.commands.intaking;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.INTAKE_RUNNER;
import frc.robot.commands.intakepivot.IntakePivotDeploy;
import frc.robot.commands.intakerunner.IntakeRunnerSetSpeed;

public class TeleopIntake extends ParallelCommandGroup {

  public TeleopIntake() {
    super(
      new IntakePivotDeploy(),
      new IntakeRunnerSetSpeed(() ->
        Value.of(
          SmartDashboard.getNumber(
            "Intake Speed",
            INTAKE_RUNNER.TELEOP_INTAKING_SPEED.in(Value)
          )
        )
      )
    );
  }
}
