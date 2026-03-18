// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SWERVE;
import frc.robot.commands.StopAllMotors;
import frc.robot.commands.controller.RumbleDriverController;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.commands.drive.DriveSetCoast;
import frc.robot.commands.drive.DriveStop;
import frc.robot.commands.floor.FloorAxis;
import frc.robot.commands.scoring.teleop.TeleopScore;
import frc.robot.commands.util.InitRobotCommand;
import frc.robot.controls.CoDriverControls;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.networkTables.AutoChooserManager;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRunner;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tunnel;
import frc.robot.triggers.ShiftWarning;
import frc.robot.vision.CameraManager;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  public static DriverControls driverControls;
  public static CoDriverControls coDriverControls;
  private static Command m_defaultDrive;

  public static CommandSwerveDrivetrain swerve;

  public static Floor floor;
  public static Alliance alliance = null;

  public static InitRobotCommand m_InitRobotCommand;

  private AutoChooserManager m_autoChooserManager;
  public static IntakeRunner intake;
  public static IntakePivot intakePivot;
  public static Shooter shooter;
  public static Hood hood;
  public static Feeder feeder;
  public static Kicker kicker;
  public static Tunnel tunnel;

  public static CameraManager cameraManager;
  public static ShotCalculator shotCalculator;

  public static ShiftTimer shiftTimer;

  private static final Field2d m_robotField = new Field2d();

  private final Telemetry logger = new Telemetry(
    Constants.SWERVE.MAX_SPEED.in(Units.MetersPerSecond)
  );

  public Robot() {
    swerve = TunerConstants.createDrivetrain();

    intake = new IntakeRunner();
    intakePivot = new IntakePivot();
    shooter = new Shooter();
    hood = new Hood();
    floor = new Floor();
    feeder = new Feeder();
    kicker = new Kicker();
    tunnel = new Tunnel();

    cameraManager = new CameraManager();

    swerve.registerTelemetry(logger::telemeterize);
    shotCalculator = new ShotCalculator();

    driverControls = new DriverControls();
    coDriverControls = new CoDriverControls();
    m_defaultDrive = new DefaultDrive(
      driverControls::getLeftX,
      driverControls::getLeftY,
      driverControls::getRightX
    );

    swerve.registerTelemetry(logger::telemeterize);
    swerve.setDefaultCommand(m_defaultDrive);

    floor.setDefaultCommand(
      new FloorAxis(() -> {
        return Value.of(SmartDashboard.getNumber("Floor", 0.0));
      })
    );
    hood.setDefaultCommand(hood.goHome());

    m_autoChooserManager = new AutoChooserManager();
    m_InitRobotCommand = new InitRobotCommand();

    shiftTimer = new ShiftTimer();

    Trigger shiftWarning = new ShiftWarning().warn();
    /** Making this trigger require being attached to the FMS to
     * avoid us getting annoyed with it rumbling
     *
     * Should remove the fms attachment requirement for drive practice
     */
    shiftWarning
      .and(DriverStation::isFMSAttached)
      .onTrue(new RumbleDriverController());

    SmartDashboard.putNumber("Floor", 0.0);
    SmartDashboard.putNumber("Shooter Target RPS", 0);

    SmartDashboard.putNumber("Hood Target Degree", 0);

    // DON'T DELETE - Load the april tag field
    // This prevents a loop overrun when we first access the constants
    AprilTagFieldLayout layout = Constants.FieldConstants.FIELD_LAYOUT;

    SmartDashboard.putData("Robot Field", m_robotField);
    SmartDashboard.putData(
      "Start",
      new TeleopScore(
        () ->
          RotationsPerSecond.of(
            SmartDashboard.getNumber("Shooter Target RPS", 0)
          ),
        () -> Degrees.of(SmartDashboard.getNumber("Hood Target Degree", 2))
      )
    );

    SmartDashboard.putNumber(
      "floor speed",
      Constants.FLOOR.FLOOR_SPEED.in(Value)
    );
    SmartDashboard.putNumber(
      "tunnel speed",
      Constants.TUNNEL.TUNNEL_SPEED.in(Value)
    );
    SmartDashboard.putNumber(
      "feed speed",
      Constants.FEEDER.FEED_SPEED_PERCENT.in(Value)
    );
    SmartDashboard.putNumber(
      "kicker speed",
      Constants.KICKER.KICK_SPEED.in(Value)
    );
  }

  @Override
  public void robotInit() {
    CommandScheduler.getInstance().schedule(m_InitRobotCommand);
  }

  @Override
  public void robotPeriodic() {
    Robot.cameraManager.updateResult();
    if (!DriverStation.isAutonomous()) {
      Robot.cameraManager.addSwerveEstimates(
        Robot.swerve::addVisionMeasurement
      );
    }
    Robot.shotCalculator.updateCalculatedShot();

    CommandScheduler.getInstance().run();

    m_robotField.setRobotPose(swerve.getFieldRelativePose2d());

    shotCalculator.updateCurveTuners();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().schedule(new StopAllMotors());

    CommandScheduler.getInstance().schedule(
      new DriveStop()
        .andThen(new WaitCommand(SWERVE.TIME_TO_COAST))
        .andThen(new DriveSetCoast())
    );

    // Log curve values when robot is disabled (like when match ends)
    shotCalculator.logCurveValues();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    swerve.configNeutralMode(NeutralModeValue.Brake);

    m_autonomousCommand = m_autoChooserManager.getSelectedCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    swerve.configNeutralMode(NeutralModeValue.Brake);
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    swerve.configNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
