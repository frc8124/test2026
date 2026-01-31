// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;

@Logged
public class Drive extends SubsystemBase {
  // The motors on the left side of the drive.
  private final SparkMax m_leftLeader;
  private final SparkMax m_leftFollower;
  private final SparkMax m_rightLeader;
  private final SparkMax m_rightFollower;

  // The robot's drive
  @NotLogged // Would duplicate motor data, there's no point sending it twice
  private final DifferentialDrive m_drive;
  //    new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

  // The left-side drive encoder
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
// Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
  private SlewRateLimiter filter = new SlewRateLimiter(0.5);
  private SlewRateLimiter filter2 = new SlewRateLimiter(1.0);

  private double slewLimit1 = 0.5;
  private double slewLimit2 = 1.0;
  private boolean curveDrive = true;
  /*   private final Encoder m_leftEncoder =
      new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);
*/

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          DriveConstants.kTurnP,
          DriveConstants.kTurnI,
          DriveConstants.kTurnD,
          new TrapezoidProfile.Constraints(
              DriveConstants.kMaxTurnRateDegPerS,
              DriveConstants.kMaxTurnAccelerationDegPerSSquared));

              private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerDegree,
          DriveConstants.kaVoltSecondsSquaredPerDegree);

  /** Creates a new Drive subsystem. */
@SuppressWarnings("removal")
public Drive() {
    m_leftLeader = new SparkMax(1, MotorType.kBrushless);
    m_leftFollower = new SparkMax(2, MotorType.kBrushless);
    m_rightLeader = new SparkMax(3, MotorType.kBrushless);
    m_rightFollower = new SparkMax(4, MotorType.kBrushless);

    m_drive = new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

    SendableRegistry.addChild(m_drive, m_leftLeader);
    SendableRegistry.addChild(m_drive, m_rightLeader);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

    /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the left leader config.
     */
    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    // Apply the global config and invert since it is on the opposite side
    rightLeaderConfig
        .apply(globalConfig)
        .inverted(true);

    // Apply the global config and set the leader SPARK for follower mode
    leftFollowerConfig
        .apply(globalConfig)
        .follow(m_leftLeader);

    // Apply the global config and set the leader SPARK for follower mode
    rightFollowerConfig
        .apply(globalConfig)
        .follow(m_rightLeader);

    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    m_leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_leftEncoder = m_leftLeader.getEncoder();
    m_rightEncoder = m_rightLeader.getEncoder();

    
    // Sets the distance per pulse for the encoders
//    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
//    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    // Set the controller to be continuous (because it is an angle controller)
    m_controller.enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    m_controller.setTolerance(
        DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  /** 
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot, boolean squareInput) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    if (curveDrive) {
    return run(() -> m_drive.curvatureDrive( filter.calculate(fwd.getAsDouble()) , filter2.calculate(rot.getAsDouble()),
      Math.abs(fwd.getAsDouble()) < 0.1 ))
        .withName("arcadeDrive");
    } else {
    return run(() -> m_drive.arcadeDrive( filter.calculate(fwd.getAsDouble()) , filter2.calculate(rot.getAsDouble()), squareInput))
        .withName("arcadeDrive");
    }

    }
  

  public Command tankDriveCommand(DoubleSupplier left, DoubleSupplier right) {
    return run( () -> m_drive.tankDrive(left.getAsDouble(), right.getAsDouble()))
        .withName("tankDrive");
  }
  
  /**
   * Returns a command that drives the robot forward a specified distance at a specified speed.
   *
   * @param distanceMeters The distance to drive forward in meters
   * @param speed The fraction of max speed at which to drive
    */
  public Command driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
            () -> {
              // Reset encoders at the start of the command
              m_leftEncoder.setPosition(0);
              m_rightEncoder.setPosition(0);
            })
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->
                Math.max(m_leftEncoder.getPosition(), m_rightEncoder.getPosition())
                    >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.stopMotor());
  }

  /**
   * Returns a command that turns to robot to the specified angle using a motion profile and PID
   * controller.
   *
   * @param angleDeg The angle to turn to
   */
  public Command turnToAngleCommand(double angleDeg) {
    return startRun(
            () -> m_controller.reset(m_gyro.getRotation2d().getDegrees()),
            () ->
                m_drive.arcadeDrive(
                    0,
                    m_controller.calculate(m_gyro.getRotation2d().getDegrees(), angleDeg)
                        // Divide feedforward voltage by battery voltage to normalize it to [-1, 1]
                        + m_feedforward.calculate(m_controller.getSetpoint().velocity)
                            / RobotController.getBatteryVoltage()))
        .until(m_controller::atGoal)
        .finallyDo(() -> m_drive.arcadeDrive(0, 0));
  }

  public Double getSlewForward() {
    return slewLimit1;
  }

  public Double getSlewRotate() {
    return slewLimit2;
  }

  /** Set the forward slew limit (units per second). Recreates the internal limiter. */
  public void setSlewForward(double limit) {
    this.slewLimit1 = limit;
    this.filter = new SlewRateLimiter(this.slewLimit1);
  }

  /** Set the rotation slew limit (units per second). Recreates the internal limiter. */
  public void setSlewRotate(double limit) {
    this.slewLimit2 = limit;
    this.filter2 = new SlewRateLimiter(this.slewLimit2);
  }

  /** Enable or disable curvature (curve) drive mode. */
  public void setCurveDrive(boolean enabled) {
    this.curveDrive = enabled;
  }

}
