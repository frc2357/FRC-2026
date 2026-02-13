// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.INTAKE_PIVOT;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakePivot extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig =
    INTAKE_PIVOT.SMC_CONFIG.withSubsystem(this);

  // Vendor motor controller object
  private SparkMax spark = new SparkMax(
    CAN_ID.INTAKE_PIVOT_MOTOR,
    MotorType.kBrushless
  );

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(
    spark,
    DCMotor.getNEO(1),
    smcConfig
  );

  // Arm Mechanism
  private Arm arm = new Arm(
    new ArmConfig(sparkSmartMotorController)
      // Soft limit is applied to the SmartMotorControllers PID
      .withSoftLimits(Units.Degrees.of(-20), Units.Degrees.of(10))
      // Hard limit is applied to the simulation.
      .withHardLimit(Units.Degrees.of(-30), Units.Degrees.of(40))
      // Starting position is where your arm starts
      .withStartingPosition(Units.Degrees.of(-5))
      // Length and mass of your arm for sim.
      .withLength(Units.Feet.of(3))
      .withMass(Units.Pounds.of(1))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("Arm", TelemetryVerbosity.HIGH)
  );

  /**
   * Set the angle of the arm, does not stop when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command setAngle(Angle angle) {
    return arm.run(angle);
  }

  /**
   * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the setpoint.
   * @param angle Angle to go to.
   * @return A Command
   */
  public Command setAngleAndStop(Angle angle) {
    return arm.runTo(angle, INTAKE_PIVOT.ANGULAR_TOLERANCE);
  }

  /**
   * Set arm closed loop controller to go to the specified mechanism position.
   * @param angle Angle to go to.
   */
  public void setAngleSetpoint(Angle angle) {
    arm.setMechanismPositionSetpoint(angle);
  }

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) {
    return arm.set(dutycycle);
  }

  public void stop() {
    arm.set(0);
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    Dimensionless m_speed = axisSpeed.times(INTAKE_PIVOT.AXIS_MAX_SPEED);
    arm.set(m_speed.in(Value));
  }

  public void setSpeed(Dimensionless dutycycle) {
    arm.set(dutycycle.in(Value));
  }

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() {
    return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }
}
