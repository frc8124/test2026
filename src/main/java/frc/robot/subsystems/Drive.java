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
// Slew rate limiters removed — direct joystick inputs are used now
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import com.revrobotics.spark.SparkClosedLoopController;

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
  // Kinematics & odometry
  private DifferentialDriveKinematics m_kinematics;
  private DifferentialDriveOdometry m_odometry;
  // Simulation state (simple kinematic sim)
  private double m_simLeftPosition = 0.0; // meters
  private double m_simRightPosition = 0.0; // meters
  private double m_simHeading = 0.0; // radians
  private double m_lastSimTimestamp = 0.0;
  // WPILib drivetrain simulator (not constructed here to avoid cross-version
  // constructor mismatches). If available in the environment, this can be
  // created and used; for now we keep the simple integrator fallback.
  private DifferentialDrivetrainSim m_driveSim = null;
  private final Field2d m_field = new Field2d();
  // (previously used slew rate limiters removed)
    private boolean curveDrive = true;

  // Slew rate limiters for operator input
  private final SlewRateLimiter m_slewForward;
  private final SlewRateLimiter m_slewRotate;
  // Track last limited values for telemetry/getters
  private double m_lastSlewForward = 0.0;
  private double m_lastSlewRotate = 0.0;
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

  // Initialize sim state from encoders/gyro so simulation starts consistent
  m_simLeftPosition = m_leftEncoder.getPosition();
  m_simRightPosition = m_rightEncoder.getPosition();
  m_simHeading = m_gyro.getRotation2d().getRadians();
  m_lastSimTimestamp = Timer.getFPGATimestamp();

  // Publish Field2d so dashboards (Glass/Shuffleboard) can display robot pose
  SmartDashboard.putData("Field", m_field);

  // Initialize kinematics and odometry (encoders configured to meters/m/s)
    m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

  // Initialize slew rate limiters
  m_slewForward = new SlewRateLimiter(DriveConstants.kSlewRateForward);
  m_slewRotate = new SlewRateLimiter(DriveConstants.kSlewRateRotate);

  // Initialize simple sim state from encoders/gyro so simulation starts consistent
  m_simLeftPosition = m_leftEncoder.getPosition();
  m_simRightPosition = m_rightEncoder.getPosition();
  m_simHeading = m_gyro.getRotation2d().getRadians();
  m_lastSimTimestamp = Timer.getFPGATimestamp();

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
      return run(() -> {
        double rawF = fwd.getAsDouble();
        double rawR = rot.getAsDouble();
        double f = m_slewForward.calculate(rawF);
        double r = m_slewRotate.calculate(rawR);
        m_lastSlewForward = f;
        m_lastSlewRotate = r;
        m_drive.curvatureDrive(f, r, Math.abs(f) < 0.1);
      }).withName("arcadeDrive");
    } else {
      return run(() -> {
        double rawF = fwd.getAsDouble();
        double rawR = rot.getAsDouble();
        double f = m_slewForward.calculate(rawF);
        double r = m_slewRotate.calculate(rawR);
        m_lastSlewForward = f;
        m_lastSlewRotate = r;
        m_drive.arcadeDrive(f, r, squareInput);
      }).withName("arcadeDrive");
    }

    }
  

  public Command tankDriveCommand(DoubleSupplier left, DoubleSupplier right) {
    return run(() -> {
      double lf = left.getAsDouble();
      double rf = right.getAsDouble();
      double l = m_slewForward.calculate(lf);
      double r = m_slewForward.calculate(rf);
          m_lastSlewForward = (Math.abs(l) > Math.abs(m_lastSlewForward)) ? l : m_lastSlewForward;
      m_drive.tankDrive(l, r);
    })
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

              // SparkMax closed-loop velocity control expects the same units as the
              // encoder velocity conversion factor. We configured the encoder
              // velocityConversionFactor to return meters/second, so pass the
              // wheel linear speed (m/s) directly.
              double leftTargetMps = -wheelLinearSpeedMPerS;
              double rightTargetMps = wheelLinearSpeedMPerS;

              try {
                m_leftLeader.getClosedLoopController().setSetpoint(
                    leftTargetMps, SparkBase.ControlType.kVelocity);
                m_rightLeader.getClosedLoopController().setSetpoint(
                    rightTargetMps, SparkBase.ControlType.kVelocity);
              } catch (Throwable t) {
                // Fallback to arcadeDrive if SparkMax velocity API isn't available at runtime
                m_drive.arcadeDrive(0, m_controller.calculate(currentAngle, angleDeg));
              }
            })
        .until(m_controller::atGoal)
        .finallyDo(() -> {
            // Stop the SparkMax velocity controllers (pass 0 m/s)
            try {
              m_leftLeader.getClosedLoopController().setSetpoint(0.0, SparkBase.ControlType.kVelocity);
              m_rightLeader.getClosedLoopController().setSetpoint(0.0, SparkBase.ControlType.kVelocity);
            } catch (Throwable t) {
              m_drive.arcadeDrive(0, 0);
            }
        });
  }

  // Slew getters removed

  @Override
  public void periodic() {
    // Publish SparkMax encoder values to SmartDashboard for debugging/tuning
    try {
      // Encoders are configured to return meters and meters/second
      SmartDashboard.putNumber("Drive/LeftEncoderPositionMeters", m_leftEncoder.getPosition());
      SmartDashboard.putNumber("Drive/LeftEncoderVelocityMPS", m_leftEncoder.getVelocity());

      SmartDashboard.putNumber("Drive/RightEncoderPositionMeters", m_rightEncoder.getPosition());
      SmartDashboard.putNumber("Drive/RightEncoderVelocityMPS", m_rightEncoder.getVelocity());

      SmartDashboard.putNumber("Drive/GyroAngleDeg", m_gyro.getRotation2d().getDegrees());
    } catch (Throwable t) {
      // If anything goes wrong (e.g., encoder not initialized yet), don't crash the robot code
    }
    // Update odometry with current gyro angle and encoder distances (meters)
    try {
      m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
      var pose = m_odometry.getPoseMeters();
      SmartDashboard.putNumber("Drive/PoseX", pose.getX());
      SmartDashboard.putNumber("Drive/PoseY", pose.getY());
      SmartDashboard.putNumber("Drive/PoseHeadingDeg", pose.getRotation().getDegrees());
      // Update Field2d robot pose for dashboards
      try {
        m_field.setRobotPose(pose);
      } catch (Throwable t) {
        // ignore if not in simulation/dashboard environment
      }
    } catch (Throwable t) {
      // ignore
    }
  }

  @Override
  public void simulationPeriodic() {
    // Simple kinematic simulation to update encoder positions and odometry.
    double now = Timer.getFPGATimestamp();
    double dt = now - m_lastSimTimestamp;
    if (dt <= 0) {
      dt = 0.02; // fallback
    }
    m_lastSimTimestamp = now;

    try {
      // If a DifferentialDrivetrainSim was constructed (environment-specific),
      // use it. Otherwise fall back to the simple integrator used previously.
      SparkClosedLoopController leftCLC = m_leftLeader.getClosedLoopController();
      SparkClosedLoopController rightCLC = m_rightLeader.getClosedLoopController();

      if (m_driveSim != null) {
        // Convert control state to voltages that the sim can consume.
        double leftVolts;
        if (leftCLC.getControlType() == SparkBase.ControlType.kVelocity) {
          double leftSetpoint = leftCLC.getSetpoint(); // m/s
          leftVolts = (leftSetpoint / DriveConstants.kMaxSpeedMetersPerSecond) * RobotController.getBatteryVoltage();
        } else {
          leftVolts = m_leftLeader.get() * RobotController.getBatteryVoltage();
        }

        double rightVolts;
        if (rightCLC.getControlType() == SparkBase.ControlType.kVelocity) {
          double rightSetpoint = rightCLC.getSetpoint();
          rightVolts = (rightSetpoint / DriveConstants.kMaxSpeedMetersPerSecond) * RobotController.getBatteryVoltage();
        } else {
          rightVolts = m_rightLeader.get() * RobotController.getBatteryVoltage();
        }

        // Step the drivetrain sim and read out wheel positions/velocities
        m_driveSim.setInputs(leftVolts, rightVolts);
        m_driveSim.update(dt);

  double leftPos = m_driveSim.getLeftPositionMeters();
  double rightPos = m_driveSim.getRightPositionMeters();
  m_leftEncoder.setPosition(leftPos);
  m_rightEncoder.setPosition(rightPos);

        // Update odometry using simulated pose
        try {
          var pose = m_driveSim.getPose();
          m_odometry.update(pose.getRotation(), leftPos, rightPos);
          m_field.setRobotPose(m_odometry.getPoseMeters());
        } catch (Throwable t) {
          // ignore if sim methods aren't available in this environment
        }
      } else {
        // Left velocity (m/s) - fallback simple integrator
        double leftVel;
        if (leftCLC.getControlType() == SparkBase.ControlType.kVelocity) {
          // setpoint is in the same units as encoder velocity conversion (m/s)
          leftVel = leftCLC.getSetpoint();
        } else {
          // Open-loop: approximate using duty cycle scaled by battery voltage
          leftVel = m_leftLeader.get() * DriveConstants.kMaxSpeedMetersPerSecond * (RobotController.getBatteryVoltage() / 12.0);
        }

        double rightVel;
        if (rightCLC.getControlType() == SparkBase.ControlType.kVelocity) {
          rightVel = rightCLC.getSetpoint();
        } else {
          rightVel = m_rightLeader.get() * DriveConstants.kMaxSpeedMetersPerSecond * (RobotController.getBatteryVoltage() / 12.0);
        }

        // Integrate positions
        m_simLeftPosition += leftVel * dt;
        m_simRightPosition += rightVel * dt;

        // Update heading (radians)
        double deltaTheta = (rightVel - leftVel) / DriveConstants.kTrackwidthMeters * dt;
        m_simHeading += deltaTheta;

        // Feed back to SparkMax encoders (we configured encoder position to meters)
        m_leftEncoder.setPosition(m_simLeftPosition);
        m_rightEncoder.setPosition(m_simRightPosition);

        
        // Update odometry using simulated gyro/encoders
        m_odometry.update(new Rotation2d(m_simHeading), m_simLeftPosition, m_simRightPosition);
        // Update Field2d (simulation)
        try {
          m_field.setRobotPose(m_odometry.getPoseMeters());
        } catch (Throwable t) {
          // ignore
        }
      }
    } catch (Throwable t) {
      // Ignore simulation failures in non-sim environments
    }
  }

  /** Returns the current robot pose as estimated by odometry. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** Returns the current wheel speeds (meters per second). */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /** Reset odometry to zero pose and zero the encoders. */
  public void resetOdometry() {
    try {
      m_leftEncoder.setPosition(0);
      m_rightEncoder.setPosition(0);
      // Reset odometry by recreating the object with zeroed distances
      m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0.0, 0.0);
    } catch (Throwable t) {
      // ignore
    }
  }

  /** Returns the last-limited forward value from the slew limiter (for telemetry). */
  public double getSlewForward() {
    return m_lastSlewForward;
  }

  /** Returns the last-limited rotate value from the slew limiter (for telemetry). */
  public double getSlewRotate() {
    return m_lastSlewRotate;
  }

}
