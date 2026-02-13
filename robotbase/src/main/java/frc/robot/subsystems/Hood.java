// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.HOOD;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class Hood extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = HOOD.SMC_CONFIG.withSubsystem(
    this
  );

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(
    CAN_ID.HOOD_MOTOR,
    MotorType.kBrushless
  );

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(
    spark,
    DCMotor.getNEO(1),
    smcConfig
  );

  // Arm Mechanism
  private Pivot pivot = new Pivot(
    HOOD.PIVOT_CONFIG.withSmartMotorController(sparkSmartMotorController)
  );

  /**
   * Set the angle of the arm, does not stop when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command setAngle(Angle angle) {
    return pivot.run(angle);
  }

  /**
   * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @return A Command
   */
  public Command setAngleAndStop(Angle angle) {
    return pivot.runTo(angle, HOOD.ANGULAR_TOLERANCE);
  }

  /**
   * Set arm closed loop controller to go to the specified mechanism position.
   * @param angle Angle to go to.
   */
  public void setAngleSetpoint(Angle angle) {
    pivot.setMechanismPositionSetpoint(angle);
  }

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) {
    return pivot.set(dutycycle);
  }

  public void stop() {
    pivot.set(0);
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    Dimensionless m_speed = axisSpeed.times(HOOD.AXIS_MAX_SPEED);
    pivot.set(m_speed.in(Value));
  }

  public void setSpeed(Dimensionless dutycycle) {
    pivot.set(dutycycle.in(Value));
  }

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() {
    return pivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    pivot.simIterate();
  }
}
