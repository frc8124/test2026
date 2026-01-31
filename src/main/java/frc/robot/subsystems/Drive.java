// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleConsumer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

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
  private boolean usePidReference = false;

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

  // Drive closed-loop / feedforward tunables (initialized from constants)
  private double driveP = DriveConstants.kDriveP;
  private double driveI = DriveConstants.kDriveI;
  private double driveD = DriveConstants.kDriveD;
  private double driveFF = DriveConstants.kDriveFF;

  // Feedforward values used for the profiled turn feedforward calculation
  private double ks = DriveConstants.ksVolts;
  private double kv = DriveConstants.kvVoltSecondsPerDegree;
  private double ka = DriveConstants.kaVoltSecondsSquaredPerDegree;
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(ks, kv, ka);
  // SparkMaxConfig instances created during construction — keep as fields so we can
  // update controller parameters at runtime by modifying and reapplying these configs.
  private SparkMaxConfig globalConfig;
  private SparkMaxConfig rightLeaderConfig;
  private SparkMaxConfig leftFollowerConfig;
  private SparkMaxConfig rightFollowerConfig;

  // We'll use the SparkMax internal PID controllers for wheel velocity control.
  // WPILib ProfiledPIDController is kept to generate the desired angular velocity
  // (deg/s) setpoint from a motion profile; we convert that to wheel velocities
  // and command the SparkMax controllers directly.

  /** Creates a new Drive subsystem. */

public Drive() {
    m_leftLeader = new SparkMax(1, MotorType.kBrushless);
    m_leftFollower = new SparkMax(2, MotorType.kBrushless);
    m_rightLeader = new SparkMax(3, MotorType.kBrushless);
    m_rightFollower = new SparkMax(4, MotorType.kBrushless);

  // Route DifferentialDrive outputs through wrapper methods so we can optionally
  // use the SparkMax closed-loop setpoint API (PID reference) instead of raw set().
      // Use DoubleConsumer wrappers so DifferentialDrive will call our
      // setLeftMotorOutput/setRightMotorOutput methods without needing a MotorController adapter.
      DoubleConsumer leftConsumer = (v) -> setLeftMotorOutput(v);
      DoubleConsumer rightConsumer = (v) -> setRightMotorOutput(v);
      m_drive = new DifferentialDrive(leftConsumer, rightConsumer);

    SendableRegistry.addChild(m_drive, m_leftLeader);
    SendableRegistry.addChild(m_drive, m_rightLeader);

  globalConfig = new SparkMaxConfig();
  rightLeaderConfig = new SparkMaxConfig();
  leftFollowerConfig = new SparkMaxConfig();
  rightFollowerConfig = new SparkMaxConfig();

    /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the left leader config.
     */
    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    
  // Configure closed-loop PID and feedforward on the SparkMax devices. These
  // values are applied to the SparkMax via the SparkMaxConfig so they persist
  // on the device.
  globalConfig.closedLoop.pid(
    DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);
  // Treat kDriveFF as the velocity feedforward (kV). Leave kS at 0.0.
  globalConfig.closedLoop.feedForward.kS(0.0);
  globalConfig.closedLoop.feedForward.kV(DriveConstants.kDriveFF);

  // Configure encoder conversion factors so RelativeEncoder returns
  // position in meters and velocity in meters/second. Account for gearbox
  // reduction: encoders are on the motor (NEO) side, so one wheel rotation is
  // (gearReduction) motor rotations.
  double wheelCircumference = Math.PI * DriveConstants.kWheelDiameterMeters;
  double posConv = wheelCircumference / DriveConstants.kDriveGearReduction; // meters per motor rotation
  double velConv = posConv / 60.0; // meters per second per RPM
  // Apply conversion factors to the SparkMax config so getPosition/getVelocity
  // return meters and meters/second respectively.
  globalConfig.encoder.positionConversionFactor((float) posConv);
  globalConfig.encoder.velocityConversionFactor((float) velConv);

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

    // Add the Drive subsystem to Shuffleboard so the sendable properties we expose
    // (Slew Forward, Slew Rotate, Curve Drive) are editable at runtime.
    try {
      Shuffleboard.getTab("Drive").add("Drive Controls", this).withSize(2, 2).withPosition(0, 0);
    } catch (Throwable t) {
      // Ignore if Shuffleboard isn't available in this environment
    }

    // Closed-loop gains are applied above via SparkMaxConfig.closedLoop so no
    // runtime setP/setI/setD calls are required here.

    
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
            () -> {
              // Get angular velocity setpoint from the profiled controller (deg/s)
              double currentAngle = m_gyro.getRotation2d().getDegrees();
              double angularSetpointDegPerS = m_controller.calculate(currentAngle, angleDeg);

              // Convert robot angular velocity (deg/s) to wheel linear speed (m/s):
              // omega_rad = deg/s * pi/180
              double omegaRadPerS = angularSetpointDegPerS * Math.PI / 180.0;
              double halfTrack = DriveConstants.kTrackwidthMeters / 2.0;
              double wheelLinearSpeedMPerS = omegaRadPerS * halfTrack;

              // Convert wheel linear speed to rotations per minute (RPM) for SparkMax setReference
              double wheelCircumference = Math.PI * DriveConstants.kWheelDiameterMeters;
              double rps = wheelLinearSpeedMPerS / wheelCircumference; // rotations per second
              double rpm = rps * 60.0;

              // Left and right wheel target RPMs: opposite signs for turning in place
              double leftTargetRPM = -rpm;
              double rightTargetRPM = rpm;

              try {
                m_leftLeader.getClosedLoopController().setSetpoint(
                    leftTargetRPM, SparkBase.ControlType.kVelocity);
                m_rightLeader.getClosedLoopController().setSetpoint(
                    rightTargetRPM, SparkBase.ControlType.kVelocity);
              } catch (Throwable t) {
                // Fallback to arcadeDrive if SparkMax velocity API isn't available at runtime
                m_drive.arcadeDrive(0, m_controller.calculate(currentAngle, angleDeg));
              }
            })
        .until(m_controller::atGoal)
        .finallyDo(() -> {
          // Stop the SparkMax velocity controllers
          try {
            m_leftLeader.getClosedLoopController().setSetpoint(0.0, SparkBase.ControlType.kVelocity);
            m_rightLeader.getClosedLoopController().setSetpoint(0.0, SparkBase.ControlType.kVelocity);
          } catch (Throwable t) {
            m_drive.arcadeDrive(0, 0);
          }
        });
  }

  public double getSlewForward() {
    return slewLimit1;
  }

  public double getSlewRotate() {
    return slewLimit2;
  }

  /** Set the forward slew limit (units per second). Recreates the internal limiter. */
  public void setSlewForward(double limit) {
    double clamped = Math.max(DriveConstants.kSlewMin, Math.min(limit, DriveConstants.kSlewMax));
    if (clamped != this.slewLimit1) {
      this.slewLimit1 = clamped;
      this.filter = new SlewRateLimiter(this.slewLimit1);
    }
  }

  /** Set the rotation slew limit (units per second). Recreates the internal limiter. */
  public void setSlewRotate(double limit) {
    double clamped = Math.max(DriveConstants.kSlewMin, Math.min(limit, DriveConstants.kSlewMax));
    if (clamped != this.slewLimit2) {
      this.slewLimit2 = clamped;
      this.filter2 = new SlewRateLimiter(this.slewLimit2);
    }
  }

  /** Enable or disable curvature (curve) drive mode. */
  public void setCurveDrive(boolean enabled) {
    this.curveDrive = enabled;
  }

  public boolean isCurveDrive() {
    return this.curveDrive;
  }

  /** Toggle whether DifferentialDrive outputs use the SparkMax PID reference API. */
  public void setUsePidReference(boolean use) {
    this.usePidReference = use;
  }

  public boolean isUsePidReference() {
    return this.usePidReference;
  }

  // Drive PID setters/getters
  public double getDriveP() {
    return this.driveP;
  }

  public void setDriveP(double p) {
    this.driveP = p;
    applyDriveClosedLoopConfig();
  }

  public double getDriveI() {
    return this.driveI;
  }

  public void setDriveI(double i) {
    this.driveI = i;
    applyDriveClosedLoopConfig();
  }

  public double getDriveD() {
    return this.driveD;
  }

  public void setDriveD(double d) {
    this.driveD = d;
    applyDriveClosedLoopConfig();
  }

  public double getDriveFF() {
    return this.driveFF;
  }

  public void setDriveFF(double ff) {
    this.driveFF = ff;
    applyDriveClosedLoopConfig();
  }

  private void applyDriveClosedLoopConfig() {
    try {
  // Update the existing config objects with the new PID/FF values and
  // reapply them to the devices. This keeps other configuration fields
  // (current limits, encoder conversions, inversion/follow) intact.
  globalConfig.closedLoop.pid((float) driveP, (float) driveI, (float) driveD);
  globalConfig.closedLoop.feedForward.kV((float) driveFF);

  // Re-derive the per-device configs and reapply
  rightLeaderConfig.apply(globalConfig);
  leftFollowerConfig.apply(globalConfig);
  rightFollowerConfig.apply(globalConfig);

  m_leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  m_rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } catch (Throwable t) {
      // Ignore if configuration cannot be applied at runtime
    }
  }

  // Feedforward constants for profiled turn feedforward
  public double getKs() { return ks; }
  public void setKs(double v) {
    this.ks = v;
    this.m_feedforward = new SimpleMotorFeedforward(this.ks, this.kv, this.ka);
  }
  public double getKv() { return kv; }
  public void setKv(double v) {
    this.kv = v;
    this.m_feedforward = new SimpleMotorFeedforward(this.ks, this.kv, this.ka);
  }
  public double getKa() { return ka; }
  public void setKa(double v) {
    this.ka = v;
    this.m_feedforward = new SimpleMotorFeedforward(this.ks, this.kv, this.ka);
  }

  // Wrapper called by the MotorController adapter for the left side. If
  // usePidReference is true we attempt to apply the value via the SparkMax
  // closed-loop setpoint API; otherwise we call set() directly.
  private void setLeftMotorOutput(double value) {
    if (usePidReference) {
      try {
        m_leftLeader.getClosedLoopController().setSetpoint(value, SparkBase.ControlType.kDutyCycle);
        return;
      } catch (Throwable t) {
        // fall through to percent output fallback
      }
    }
    m_leftLeader.set(value);
  }

  // Wrapper called by the MotorController adapter for the right side.
  private void setRightMotorOutput(double value) {
    if (usePidReference) {
      try {
        m_rightLeader.getClosedLoopController().setSetpoint(value, SparkBase.ControlType.kDutyCycle);
        return;
      } catch (Throwable t) {
        // fall through to percent output fallback
      }
    }
    m_rightLeader.set(value);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Expose the slew limits and curveDrive toggle to Shuffleboard/SmartDashboard so
    // they can be adjusted at runtime during teleop for tuning.
    builder.addDoubleProperty("Slew Forward", this::getSlewForward, this::setSlewForward);
    builder.addDoubleProperty("Slew Rotate", this::getSlewRotate, this::setSlewRotate);
    builder.addBooleanProperty("Curve Drive", () -> this.curveDrive, this::setCurveDrive);
    builder.addBooleanProperty("Use PID Reference", this::isUsePidReference, this::setUsePidReference);
    // Drive controller PID and feedforward tunables
    builder.addDoubleProperty("Drive P", () -> this.driveP, this::setDriveP);
    builder.addDoubleProperty("Drive I", () -> this.driveI, this::setDriveI);
    builder.addDoubleProperty("Drive D", () -> this.driveD, this::setDriveD);
    builder.addDoubleProperty("Drive FF", () -> this.driveFF, this::setDriveFF);
    // Turn/feedforward constants
    builder.addDoubleProperty("Feedforward kS", () -> this.ks, this::setKs);
    builder.addDoubleProperty("Feedforward kV", () -> this.kv, this::setKv);
    builder.addDoubleProperty("Feedforward kA", () -> this.ka, this::setKa);
  }

  @Override
  public void periodic() {
    // Publish SparkMax encoder values to SmartDashboard for debugging/tuning
    try {
      SmartDashboard.putNumber("Drive/LeftEncoderPositionRotations", m_leftEncoder.getPosition());
      SmartDashboard.putNumber("Drive/LeftEncoderVelocityRPM", m_leftEncoder.getVelocity());

      SmartDashboard.putNumber("Drive/RightEncoderPositionRotations", m_rightEncoder.getPosition());
      SmartDashboard.putNumber("Drive/RightEncoderVelocityRPM", m_rightEncoder.getVelocity());

      SmartDashboard.putNumber("Drive/GyroAngleDeg", m_gyro.getRotation2d().getDegrees());
    } catch (Throwable t) {
      // If anything goes wrong (e.g., encoder not initialized yet), don't crash the robot code
    }
  }

}
