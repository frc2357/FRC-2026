// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.HOOD;
import frc.robot.Robot;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Hood extends SubsystemBase {

  private final SparkMax m_hoodMotor = new SparkMax(CAN_ID.HOOD_MOTOR, MotorType.kBrushless);

  private final SmartMotorControllerConfig m_hoodMotorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(
          50,
          0,
          0,
          Units.DegreesPerSecond.of(90),
          Units.DegreesPerSecondPerSecond.of(45)
        )
        .withSimClosedLoopController(
          50,
          0,
          0,
          Units.DegreesPerSecond.of(90),
          Units.DegreesPerSecondPerSecond.of(45)
        )
        .withFeedforward(new ArmFeedforward(0, 0, 0))
        .withSimFeedforward(new ArmFeedforward(0, 0, 0))
        .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
        // Gearing from the motor rotor to final shaft.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Units.Amps.of(40))
        .withClosedLoopRampRate(Units.Seconds.of(0.25))
        .withOpenLoopRampRate(Units.Seconds.of(0.25));

  private final SmartMotorController m_hoodSMC = new SparkWrapper(m_hoodMotor, DCMotor.getNeo550(1), m_hoodMotorConfig);

  private final ArmConfig m_hoodConfig = new ArmConfig(m_hoodSMC)
          // Soft limit is applied to the SmartMotorControllers PID
      .withSoftLimits(Units.Degrees.of(-20), Units.Degrees.of(10))
      // Hard limit is applied to the simulation.
      .withHardLimit(Units.Degrees.of(-30), Units.Degrees.of(40))
      // Starting position is where your arm starts
      .withStartingPosition(Units.Degrees.of(-5))
      // Length and mass of your arm for sim.
      .withLength(Units.Inches.of(7)) // 7 is an estimate
      .withMass(Units.Pounds.of(1.365))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);


  private final Arm m_hood = new Arm(m_hoodConfig);

  private final SparkAbsoluteEncoder m_encoder1 = m_hoodMotor.getAbsoluteEncoder();
  private final SparkAbsoluteEncoder m_encoder2 = Robot.shooter.getSecondHoodEncoder();

  private final EasyCRTConfig m_crtConfig = new EasyCRTConfig(() -> Units.Rotations.of(m_encoder1.getPosition()), () -> Units.Rotations.of(m_encoder2.getPosition()))
    .withAbsoluteEncoder1GearingStages(20,19,16,266)
    .withAbsoluteEncoder2GearingStages(16,266)
    .withMechanismRange(Units.Degrees.of(0), Units.Degrees.of(30));

  private final EasyCRT m_crtSolver = new EasyCRT(m_crtConfig);

  public Hood() {

  }

      public Command setAngle(Supplier<Angle> angleSupplier) {
        return m_hood.setAngle(angleSupplier);
    }

    public Angle getAngle() {
        return m_hood.getAngle();
    }

    public Command sysId() {
        return m_hood.sysId(
                Units.Volts.of(4.0), // maximumVoltage
                Units.Volts.per(Units.Second).of(0.5), // step
                Units.Seconds.of(8.0) // duration
        );
    }

    public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
        return m_hood.set(dutyCycleSupplier);
    }

    public Command setDutyCycle(double dutyCycle) {
        return m_hood.set(dutyCycle);
    }

      public void stop() {
    m_hood.set(0);
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    Dimensionless m_speed = axisSpeed.times(HOOD.AXIS_MAX_SPEED);
    m_hood.set(m_speed.in(Units.Value));
  }

  public void setSpeed(Dimensionless dutycycle) {
    m_hood.set(dutycycle.in(Units.Value));
  }

  /**
   * The main purpose of this method is so we can tell if the hood is stationary
   * The CRT calculations can get messed up if the mechanism is moving
   */
  public AngularVelocity getMotorVelocity() {
    return RPM.of(m_hoodMotor.getEncoder().getVelocity());
  }

  public boolean zero() {
    Optional<Angle> crtAngle = m_crtSolver.getAngleOptional();

    if (crtAngle.isPresent()) {
      m_hoodSMC.setEncoderPosition(crtAngle.get());
      return true;
    } else {
      return false;
    }
  }

    @Override
    public void periodic() {
        m_hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_hood.simIterate();
    }
}
