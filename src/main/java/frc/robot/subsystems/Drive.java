// ...existing code...
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
  private boolean curveDrive = true;
  private boolean usePidReference = false;

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
              double wheelCircumference = Math.PI * DriveConstants.kWheelDiameterMeters;
              double rps = wheelLinearSpeedMPerS / wheelCircumference; // rotations per second
              double rpm = rps * 60.0;
              double leftTargetRPM = -rpm;
              double rightTargetRPM = rpm;
              try {
                m_leftLeader.getClosedLoopController().setSetpoint(
                    leftTargetRPM, SparkBase.ControlType.kVelocity);
                m_rightLeader.getClosedLoopController().setSetpoint(
                    rightTargetRPM, SparkBase.ControlType.kVelocity);
              } catch (Throwable t) {
                m_drive.arcadeDrive(0, m_controller.calculate(currentAngle, angleDeg));
              }
            })
        .until(m_controller::atGoal)
        .finallyDo(() -> {
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
// ...existing code...
