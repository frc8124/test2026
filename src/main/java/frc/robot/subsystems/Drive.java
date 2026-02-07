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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.RobotBase;

@Logged
public class Drive extends SubsystemBase {
  // Adapter interfaces to decouple code from real vendor APIs for simulation.
  private interface EncoderAdapter {
    double getPosition();
    double getVelocity();
    void setPosition(double pos);
  }

  private interface ClosedLoopAdapter {
    void setSetpoint(double value, Object controlType);
  }

  private interface MotorAdapter {
    void set(double v);
    EncoderAdapter getEncoder();
    ClosedLoopAdapter getClosedLoopController();
    void configure(SparkMaxConfig cfg, ResetMode resetMode, PersistMode persistMode);
    boolean isReal(); // true if wraps a real SparkMax
    SparkMax getWrappedSparkMax(); // may return null for sim
  }

  // Real motor wrapper that delegates to SparkMax (used when running on real robot and vendordeps present)
  private static class RealMotorAdapter implements MotorAdapter {
    private final SparkMax spark;
    RealMotorAdapter(SparkMax s) { this.spark = s; }
    @Override public void set(double v) { spark.set(v); }
    @Override public EncoderAdapter getEncoder() {
      RelativeEncoder e = spark.getEncoder();
      return new EncoderAdapter() {
        @Override public double getPosition() { return e.getPosition(); }
        @Override public double getVelocity() { return e.getVelocity(); }
        @Override public void setPosition(double pos) { e.setPosition(pos); }
      };
    }
    @Override public ClosedLoopAdapter getClosedLoopController() {
      final var cl = spark.getClosedLoopController();
      return (value, controlType) -> {
        try {
          // Expect controlType to be SparkBase.ControlType
          cl.setSetpoint(value, (SparkBase.ControlType) controlType);
        } catch (Throwable t) {
          // ignore in case of incompatible runtime
        }
      };
    }
    @Override public void configure(SparkMaxConfig cfg, ResetMode resetMode, PersistMode persistMode) {
      spark.configure(cfg, resetMode, persistMode);
    }
    @Override public boolean isReal() { return true; }
    @Override public SparkMax getWrappedSparkMax() { return spark; }
  }

  // Lightweight software simulation adapter used when running in simulation (no vendor sim dependency required).
  private static class SimMotorAdapter implements MotorAdapter {
    private final SimEncoder enc = new SimEncoder();
    private double output = 0.0;
    @Override public void set(double v) { this.output = v; /* simple model: velocity proportional to output */ enc.velocity = v * 100.0; }
    @Override public EncoderAdapter getEncoder() { return enc; }
    @Override public ClosedLoopAdapter getClosedLoopController() {
      return (value, controlType) -> {
        // very small simulated effect: treat value as velocity or duty cycle depending on controlType name if available
        enc.velocity = value;
      };
    }
    @Override public void configure(SparkMaxConfig cfg, ResetMode resetMode, PersistMode persistMode) { /* no-op in sim */ }
    @Override public boolean isReal() { return false; }
    @Override public SparkMax getWrappedSparkMax() { return null; }

    private static class SimEncoder implements EncoderAdapter {
      private double position = 0.0; // rotations or meters depending on how used
      private double velocity = 0.0; // RPM or m/s-ish depending
      @Override public double getPosition() { return position; }
      @Override public double getVelocity() { return velocity; }
      @Override public void setPosition(double pos) { this.position = pos; }
    }
  }

  // The motors on the left side of the drive.
  private final MotorAdapter m_leftLeader;
  private final MotorAdapter m_leftFollower;
  private final MotorAdapter m_rightLeader;
  private final MotorAdapter m_rightFollower;

  // The robot's drive
  @NotLogged // Would duplicate motor data, there's no point sending it twice
  private final DifferentialDrive m_drive;

  // The left-side drive encoder
  private final EncoderAdapter m_leftEncoder;
  private final EncoderAdapter m_rightEncoder;

  // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
  private SlewRateLimiter filter = new SlewRateLimiter(0.5);
  private SlewRateLimiter filter2 = new SlewRateLimiter(1.0);

  private double slewLimit1 = 0.5;
  private double slewLimit2 = 1.0;

  private boolean usePidReference = false;
//  private final RelativeEncoder m_leftEncoder;
//  private final RelativeEncoder m_rightEncoder;
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

  private double ks = DriveConstants.ksVolts;
  private double kv = DriveConstants.kvVoltSecondsPerDegree;
  private double ka = DriveConstants.kaVoltSecondsSquaredPerDegree;
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(ks, kv, ka);

  private SparkMaxConfig globalConfig;
  private SparkMaxConfig rightLeaderConfig;
  private SparkMaxConfig leftFollowerConfig;
  private SparkMaxConfig rightFollowerConfig;

  public Drive() {
    // Create real adapters if running on robot and vendor libs available; otherwise create simulation adapters.
    MotorAdapter leftL = null, leftF = null, rightL = null, rightF = null;
    try {
      if (RobotBase.isReal()) {
        // real robot: construct actual SparkMax instances and wrap them
        leftL = new RealMotorAdapter(new SparkMax(1, MotorType.kBrushless));
        leftF = new RealMotorAdapter(new SparkMax(2, MotorType.kBrushless));
        rightL = new RealMotorAdapter(new SparkMax(3, MotorType.kBrushless));
        rightF = new RealMotorAdapter(new SparkMax(4, MotorType.kBrushless));
      } else {
        // simulation: use lightweight adapters (no vendor simulation dependency required)
        leftL = new SimMotorAdapter();
        leftF = new SimMotorAdapter();
        rightL = new SimMotorAdapter();
        rightF = new SimMotorAdapter();
      }
    } catch (Throwable t) {
      // Fallback to simulation adapters if any vendor construction fails
      leftL = new SimMotorAdapter();
      leftF = new SimMotorAdapter();
      rightL = new SimMotorAdapter();
      rightF = new SimMotorAdapter();
    }

    m_leftLeader = leftL;
    m_leftFollower = leftF;
    m_rightLeader = rightL;
    m_rightFollower = rightF;

    // Route DifferentialDrive outputs through wrapper methods so we can optionally
    // use the SparkMax closed-loop setpoint API (PID reference) instead of raw set().
    DoubleConsumer leftConsumer = (v) -> setLeftMotorOutput(v);
    DoubleConsumer rightConsumer = (v) -> setRightMotorOutput(v);
    m_drive = new DifferentialDrive(leftConsumer, rightConsumer);

    // If we have real SparkMax instances, add them as children for SendableRegistry.
    try {
      if (m_leftLeader.isReal()) {
        SendableRegistry.addChild(m_drive, m_leftLeader.getWrappedSparkMax());
      }
      if (m_rightLeader.isReal()) {
        SendableRegistry.addChild(m_drive, m_rightLeader.getWrappedSparkMax());
      }
    } catch (Throwable ignored) {}

    globalConfig = new SparkMaxConfig();
    rightLeaderConfig = new SparkMaxConfig();
    leftFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig = new SparkMaxConfig();

    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    globalConfig.closedLoop.pid(
      DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);
    globalConfig.closedLoop.feedForward.kS(0.0);
    globalConfig.closedLoop.feedForward.kV(DriveConstants.kDriveFF);

    double wheelCircumference = Math.PI * DriveConstants.kWheelDiameterMeters;
    double posConv = wheelCircumference / DriveConstants.kDriveGearReduction; // meters per motor rotation
    double velConv = posConv / 60.0; // meters per second per RPM
    globalConfig.encoder.positionConversionFactor((float) posConv);
    globalConfig.encoder.velocityConversionFactor((float) velConv);

    rightLeaderConfig
        .apply(globalConfig)
        .inverted(true);

    leftFollowerConfig
        .apply(globalConfig)
        .follow(m_leftLeader.isReal() ? m_leftLeader.getWrappedSparkMax() : null);

    rightFollowerConfig
        .apply(globalConfig)
        .follow(m_rightLeader.isReal() ? m_rightLeader.getWrappedSparkMax() : null);

    try {
      // Apply the configuration to the SPARKs only when real adapters are used
      if (m_leftLeader.isReal()) m_leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      if (m_leftFollower.isReal()) m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      if (m_rightLeader.isReal()) m_rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      if (m_rightFollower.isReal()) m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    } catch (Throwable ignored) {}

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

  // Initialize simple sim state from encoders/gyro so simulation starts consistent
  m_simLeftPosition = m_leftEncoder.getPosition();
  m_simRightPosition = m_rightEncoder.getPosition();
  m_simHeading = m_gyro.getRotation2d().getRadians();
  m_lastSimTimestamp = Timer.getFPGATimestamp();

    try {
      Shuffleboard.getTab("Drive").add("Drive Controls", this).withSize(2, 2).withPosition(0, 0);
    } catch (Throwable t) {
      // Ignore if Shuffleboard isn't available in this environment
    }

    m_controller.enableContinuousInput(-180, 180);
    m_controller.setTolerance(
      DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  // ...existing command methods unchanged (they will call through adapters)...
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot, boolean squareInput) {
    if (curveDrive) {
      return run(() -> {
        double rawF = fwd.getAsDouble();
        double rawR = rot.getAsDouble();
        double f = m_slewForward.calculate(rawF);
        double r = m_slewRotate.calculate(rawR);
        m_drive.curvatureDrive(f, r, Math.abs(f) < 0.1);
      }).withName("arcadeDrive");
    } else {
      return run(() -> {
        double rawF = fwd.getAsDouble();
        double rawR = rot.getAsDouble();
        double f = m_slewForward.calculate(rawF);
        double r = m_slewRotate.calculate(rawR);
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
              try { m_leftEncoder.setPosition(0); } catch (Throwable ignored) {}
              try { m_rightEncoder.setPosition(0); } catch (Throwable ignored) {}
            })
        .andThen(run(() -> m_drive.arcadeDrive(speed, 0)))
        .until(
            () ->
                Math.max(safeGet(m_leftEncoder::getPosition), safeGet(m_rightEncoder::getPosition))
                    >= distanceMeters)
        .finallyDo(interrupted -> m_drive.stopMotor());
  }

  public Command turnToAngleCommand(double angleDeg) {
    return startRun(
            () -> m_controller.reset(m_gyro.getRotation2d().getDegrees()),
            () -> {
              double currentAngle = m_gyro.getRotation2d().getDegrees();
              double angularSetpointDegPerS = m_controller.calculate(currentAngle, angleDeg);
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

  private static double safeGet(java.util.function.Supplier<Double> s) {
    try { return s.get(); } catch (Throwable t) { return 0.0; }
  }

  public double getSlewForward() { return slewLimit1; }
  public double getSlewRotate() { return slewLimit2; }
  public void setSlewForward(double limit) {
    double clamped = Math.max(DriveConstants.kSlewMin, Math.min(limit, DriveConstants.kSlewMax));
    if (clamped != this.slewLimit1) {
      this.slewLimit1 = clamped;
      this.filter = new SlewRateLimiter(this.slewLimit1);
    }
  }
  public void setSlewRotate(double limit) {
    double clamped = Math.max(DriveConstants.kSlewMin, Math.min(limit, DriveConstants.kSlewMax));
    if (clamped != this.slewLimit2) {
      this.slewLimit2 = clamped;
      this.filter2 = new SlewRateLimiter(this.slewLimit2);
    }
  }
  public void setCurveDrive(boolean enabled) { this.curveDrive = enabled; }
  public boolean isCurveDrive() { return this.curveDrive; }
  public void setUsePidReference(boolean use) { this.usePidReference = use; }
  public boolean isUsePidReference() { return this.usePidReference; }

  public double getDriveP() { return this.driveP; }
  public void setDriveP(double p) { this.driveP = p; applyDriveClosedLoopConfig(); }
  public double getDriveI() { return this.driveI; }
  public void setDriveI(double i) { this.driveI = i; applyDriveClosedLoopConfig(); }
  public double getDriveD() { return this.driveD; }
  public void setDriveD(double d) { this.driveD = d; applyDriveClosedLoopConfig(); }
  public double getDriveFF() { return this.driveFF; }
  public void setDriveFF(double ff) { this.driveFF = ff; applyDriveClosedLoopConfig(); }

  private void applyDriveClosedLoopConfig() {
    try {
      globalConfig.closedLoop.pid((float) driveP, (float) driveI, (float) driveD);
      globalConfig.closedLoop.feedForward.kV((float) driveFF);

      rightLeaderConfig.apply(globalConfig);
      leftFollowerConfig.apply(globalConfig);
      rightFollowerConfig.apply(globalConfig);

      if (m_leftLeader.isReal()) m_leftLeader.configure(globalConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      if (m_leftFollower.isReal()) m_leftFollower.configure(leftFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      if (m_rightLeader.isReal()) m_rightLeader.configure(rightLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      if (m_rightFollower.isReal()) m_rightFollower.configure(rightFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    } catch (Throwable t) {
      // Ignore if configuration cannot be applied at runtime
    }
  }

  public double getKs() { return ks; }
  public void setKs(double v) { this.ks = v; this.m_feedforward = new SimpleMotorFeedforward(this.ks, this.kv, this.ka); }
  public double getKv() { return kv; }
  public void setKv(double v) { this.kv = v; this.m_feedforward = new SimpleMotorFeedforward(this.ks, this.kv, this.ka); }
  public double getKa() { return ka; }
  public void setKa(double v) { this.ka = v; this.m_feedforward = new SimpleMotorFeedforward(this.ks, this.kv, this.ka); }

  private void setLeftMotorOutput(double value) {
    if (usePidReference) {
      try {
        m_leftLeader.getClosedLoopController().setSetpoint(value, SparkBase.ControlType.kDutyCycle);
        return;
      } catch (Throwable t) {
        // fall through to percent output fallback
      }
    }
    try { m_leftLeader.set(value); } catch (Throwable ignored) {}
  }

  private void setRightMotorOutput(double value) {
    if (usePidReference) {
      try {
        m_rightLeader.getClosedLoopController().setSetpoint(value, SparkBase.ControlType.kDutyCycle);
        return;
      } catch (Throwable t) {
        // fall through to percent output fallback
      }
    }
    try { m_rightLeader.set(value); } catch (Throwable ignored) {}
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Slew Forward", this::getSlewForward, this::setSlewForward);
    builder.addDoubleProperty("Slew Rotate", this::getSlewRotate, this::setSlewRotate);
    builder.addBooleanProperty("Curve Drive", () -> this.curveDrive, this::setCurveDrive);
    builder.addBooleanProperty("Use PID Reference", this::isUsePidReference, this::setUsePidReference);
    builder.addDoubleProperty("Drive P", () -> this.driveP, this::setDriveP);
    builder.addDoubleProperty("Drive I", () -> this.driveI, this::setDriveI);
    builder.addDoubleProperty("Drive D", () -> this.driveD, this::setDriveD);
    builder.addDoubleProperty("Drive FF", () -> this.driveFF, this::setDriveFF);
    builder.addDoubleProperty("Feedforward kS", () -> this.ks, this::setKs);
    builder.addDoubleProperty("Feedforward kV", () -> this.kv, this::setKv);
    builder.addDoubleProperty("Feedforward kA", () -> this.ka, this::setKa);
  }

  @Override
  public void periodic() {
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

}
