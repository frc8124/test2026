// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// REV simulation helpers
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

@Logged
public class Shooter extends SubsystemBase {

  private SparkMax m_shooterMotor;
 // private SparkMax m_feederMotor;


private RelativeEncoder m_shooterEncoder;
  // WPILib flywheel sim (used to model motor + inertia)
  private FlywheelSim m_flywheelSim;
  private double m_simPosition = 0.0;
  private double m_simVelocity = 0.0;
  private double m_lastSimTimestamp = 0.0;
  // REV simulation helpers
  private SparkMaxSim m_shooterSim = null;

       private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
     private final SparkMaxConfig feederMotorConfig = new SparkMaxConfig();

     private double m_setpoint = 0.0;

  /** The shooter subsystem for the robot. */
  public Shooter() {

     SparkMaxConfig globalConfig = new SparkMaxConfig();
   
    globalConfig
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    globalConfig.closedLoop.pid(ShooterConstants.kP, 0.0, ShooterConstants.kD);
    globalConfig.closedLoop.allowedClosedLoopError(ShooterConstants.kShooterToleranceRPS,ClosedLoopSlot.kSlot0);
    globalConfig.closedLoop.feedForward.kS(ShooterConstants.kSVolts);
    globalConfig.closedLoop.feedForward.kV(ShooterConstants.kVVoltSecondsPerRotation);
       
    globalConfig.encoder.countsPerRevolution(ShooterConstants.kEncoderCPR);
    globalConfig.encoder.positionConversionFactor((float) 1.0); // to get revolutions of flywheel per pulse
    globalConfig.encoder.velocityConversionFactor((float) 1.0 / 60.0); // revs per second
    
    // Encoder appears inverted on our test rig.
    globalConfig.encoder.inverted(false);

     shooterMotorConfig
        .apply(globalConfig)
        .inverted(false);


    feederMotorConfig
        .apply(globalConfig)
        .inverted(false);


  m_shooterMotor = new SparkMax(DriveConstants.kShooterMotorID, MotorType.kBrushed);
 // m_feederMotor = new SparkMax(DriveConstants.kFeederMotorID, MotorType.kBrushed);
 //   m_shooterFeedback.setTolerance(ShooterConstants.kShooterToleranceRPS);
   // m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
  m_shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  
  m_shooterMotor.set(0);

  // Initialize shooter encoder wrapper from the motor controller
  try { m_shooterEncoder = m_shooterMotor.getEncoder(); } catch (Throwable ignored) {}
    // Setup simulation helpers when running in simulation
    if (RobotBase.isSimulation()) {
      try {
        // Create FlywheelSim using a simple motor + inertia model
        final double gearing = 1.0;
        final double flywheelMOI = 0.000103; // 10cmx1cm ABS plastic
        LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(DCMotor.getCIM(1), gearing, flywheelMOI);
        m_flywheelSim = new FlywheelSim(plant, DCMotor.getCIM(1));

        // Create REV simulation helpers and attach them to objects
        try {
          m_shooterSim = new SparkMaxSim(m_shooterMotor, DCMotor.getCIM(1));
        } catch (Throwable ignored) {
          m_shooterSim = null;
        }

        } catch (Throwable ignored) {
          // If any sim classes are missing or incompatible, just skip sim wiring.
          m_flywheelSim = null;
          m_shooterSim = null;
        }
      }

 // m_feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    // Set default command to turn off both the shooter and feeder motors, and then idle
    setDefaultCommand(
        runOnce(
                () -> {
                  this.setSetpoint(0.0);
                  m_shooterMotor.set(0); // disable();
                 // m_feederMotor.disable();
                })
            .withName("Idle"));
  }

  /**
   * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
   * up to the specified setpoint, and then runs the feeder motor.
   *
   * @param setpointRotationsPerSecond The desired shooter velocity
   */
  public Command shootCommand(double setpointRotationsPerSecond) {
    return // parallel(
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                 this.setSetpoint(setpointRotationsPerSecond);
                 m_shooterMotor.getClosedLoopController().setSetpoint(setpointRotationsPerSecond,  SparkBase.ControlType.kVelocity);
                }
            )
            // .until( () -> m_shooterMotor.getClosedLoopController().isAtSetpoint())
        .withName("Shoot");
 //    );
  }

  private void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
  }

  @Override
  public void periodic() {
    try {
      double pos = (m_shooterEncoder != null) ? m_shooterEncoder.getPosition() : 0.0;
      SmartDashboard.putNumber("Shooter/SetPoint", m_setpoint);
      SmartDashboard.putNumber("Shooter/EncoderPosition", pos);
      SmartDashboard.putNumber("Shooter/EncoderVelocity",  m_shooterEncoder.getVelocity());
      SmartDashboard.putNumber("Shooter/SimPosition", m_simPosition);
      SmartDashboard.putNumber("Shooter/SimVelocity", m_simVelocity);
    } catch (Throwable t) {
      // ignore errors in dashboard publishing
    }
  }

  @Override
  public void simulationPeriodic() {
    double now = Timer.getFPGATimestamp();
    double dt = now - m_lastSimTimestamp;  // Advance the flywheel sim and update REV encoder sim helpers when running in simulation.
  
        SmartDashboard.putNumber("Shooter/dt", dt);
  
      m_lastSimTimestamp = now;

    if (m_flywheelSim == null || m_shooterMotor == null) {
      return;
    }

    double motorVoltage = RobotController.getBatteryVoltage();

    try {
      m_flywheelSim.setInputVoltage(motorVoltage);
      m_flywheelSim.update(dt);
      double angVelRadPerSec = m_flywheelSim.getAngularVelocityRadPerSec();
      m_simVelocity = angVelRadPerSec / (2.0 * Math.PI);
      m_simPosition += m_simVelocity * 0.02;
    } catch (Throwable ignored) {
      // fallback simple model
    }

    // this updates the simulated encoder too
    m_shooterSim.iterate(m_simVelocity, motorVoltage, dt);
  }
}
