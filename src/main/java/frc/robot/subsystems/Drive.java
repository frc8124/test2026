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
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private final SlewRateLimiter filter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter filter2 = new SlewRateLimiter(1.0);

  private Double slewLimit1 = 0.5;
  private Double slewLimit2 = 0.5;
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

  // We'll use the SparkMax internal PID controllers for wheel velocity control.
  // WPILib ProfiledPIDController is kept to generate the desired angular velocity
  // (deg/s) setpoint from a motion profile; we convert that to wheel velocities
  // and command the SparkMax controllers directly.

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

  public Double getSlewForward() {
    return slewLimit1;
  }

  public Double getSlewRotate() {
    return slewLimit2;
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
