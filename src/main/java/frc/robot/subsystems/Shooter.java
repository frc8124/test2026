// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

// REV simulation helpers
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
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
private boolean m_ballInShooter = false;
private Boolean m_lastBallInShooter = false;
private int m_ballCountDebounce = 0;

  public boolean ballInShooter() {
  return m_ballInShooter;
}

  private SparkMax m_shooterMotor;
  private RelativeEncoder m_shooterEncoder;
  // REV 2m distance sensor (mounted near shooter to measure target distance)
  private Rev2mDistanceSensor m_rev2m;
  
  // WPILib flywheel sim (used to model motor + inertia)
  private FlywheelSim m_flywheelSim;
  private double m_simVelocity = 0.0;
  private double m_lastSimTimestamp = 0.0;
  private SparkMaxSim m_shooterSim = null;
  private static final double simStaticV = 0.004;
 
  private int m_ballCount = 0;

  private boolean m_faultPresent = false;

  private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();

  // private SparkMax m_feederMotor;
  //  private final SparkMaxConfig feederMotorConfig = new SparkMaxConfig();

  /** The shooter subsystem for the robot. */
  public Shooter() {

     SparkMaxConfig globalConfig = new SparkMaxConfig();
   
    globalConfig
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kCoast);

       
    globalConfig.encoder.countsPerRevolution(ShooterConstants.kEncoderCPR);
    globalConfig.encoder.positionConversionFactor((float) 1.0); // to get revolutions of flywheel per pulse
    globalConfig.encoder.velocityConversionFactor((float) 1.0); // revs per minute
    
    shooterMotorConfig
      .apply(globalConfig)
      .inverted(true)
      .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.kP, 0.0, ShooterConstants.kD)
        .allowedClosedLoopError(ShooterConstants.KToleranceRPM,ClosedLoopSlot.kSlot0)
        .outputRange(-1,1)
        .feedForward.kS(ShooterConstants.kSVolts)
          .kV(ShooterConstants.kVVoltSecondsPerRotation);

    shooterMotorConfig.encoder.inverted(ShooterConstants.kEncoderReversed);

  /*
  // Initialize feeder motor
    feederMotorConfig
        .apply(globalConfig)
        .inverted(false);
  m_feederMotor = new SparkMax(DriveConstants.kFeederMotorID, MotorType.kBrushed);
  m_feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); //uncomment to add feeder motor back in
  */

  m_shooterMotor = new SparkMax(ShooterConstants.kMotorID, MotorType.kBrushed);
 
  m_shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  m_shooterMotor.disable();

  // Initialize shooter encoder wrapper from the motor controller
  m_shooterEncoder = m_shooterMotor.getEncoder();
    // Initialize REV 2m distance sensor (onboard port) if available
    try {
      m_rev2m = new Rev2mDistanceSensor(Port.kOnboard);
      // enable automatic ranging if supported
      m_rev2m.setMeasurementPeriod(0.02);
      try {
        m_rev2m.setAutomaticMode(true);
      } catch (Throwable ignore) {
        // optional
      }
    } catch (Throwable t) {
      m_rev2m = null; // sensor not available or failed to initialize
    }
    
  // Setup simulation helpers when running in simulation
  if (RobotBase.isSimulation()) {
    try {
      // Create FlywheelSim using a simple motor + inertia model
      final double gearing = 1.0;
      final double flywheelMOI = 0.000103 * 2; // 10cmx1cm ABS plastic x 10
      DCMotor gearbox = DCMotor.getCIM(1);
      LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(gearbox, flywheelMOI, gearing);
      m_flywheelSim = new FlywheelSim(plant, gearbox);

      // Create REV simulation helpers and attach them to objects
      m_shooterSim = new SparkMaxSim(m_shooterMotor, gearbox);

      } catch (Throwable ignored) {
        // If any sim classes are missing or incompatible, just skip sim wiring.
        m_flywheelSim = null;
        m_shooterSim = null;
      }
    }

    // Set default command to turn off both the shooter and feeder motors, and then idle
    setDefaultCommand(
        runOnce(
                () -> {
                  m_shooterMotor.disable();
                  // m_feederMotor.disable(); // disable(); feeder motor disabled
                })
            .withName("Idle"));
  }
  
public boolean faultPresent() {

    return m_faultPresent; // m_shooterEncoder.getVelocity() < 100;
 }

 private void setFaultPresent() {
  m_faultPresent = true;
 }

public Command stopCommand() {
  return run (() -> {m_shooterMotor.disable(); }).withName ("Stop");
}



  /**
   * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
   * up to the specified setpoint, and then runs the feeder motor.
   *
   */
  public Command shootCommand() {
    return sequence(
            new InstantCommand( () -> { resetFault(); } ),       
    // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                 m_shooterMotor.getClosedLoopController().setSetpoint(ShooterConstants.kTargetRPM,  SparkBase.ControlType.kVelocity);
                //m_feederMotor.getClosedLoopController().setSetpoint(setpointRotationsPerSecondFeeder,  SparkBase.ControlType.kVelocity); //uncomment to add feeder motor back in
                }
            ).withTimeout(2.5)
            .finallyDo( () -> { setFaultPresent(); } )
            // .until( () -> m_shooterMotor.getClosedLoopController().isAtSetpoint())
    )
        .withName("Shoot");

        
 //    );
  }


public Command unstickCommand() {
    return sequence(
            new InstantCommand( () -> { resetFault(); } ),       
    // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                 m_shooterMotor.set(1);
                }
            ).withTimeout(1.0),
 //run(
 //               () -> {
 //                m_shooterMotor.set(-1.0);
 //               }
 //          ).withTimeout(1.0),
          runOnce( () -> { m_shooterMotor.disable(); } )
    )
        .withName("Unstick");
        
 //    );
  }

public Command unloadCommand() {
  return 
    run(
        () -> {
          m_shooterMotor.getClosedLoopController().setSetpoint(-ShooterConstants.kIntakeRPM,  SparkBase.ControlType.kVelocity);
        //m_feederMotor.getClosedLoopController().setSetpoint(setpointRotationsPerSecondFeeder,  SparkBase.ControlType.kVelocity); //uncomment to add feeder motor back in
        }
    ).withName("Unload");
  }

  public Command intakeCommand() {
  return 
    run(
        () -> {
          m_shooterMotor.getClosedLoopController().setSetpoint(ShooterConstants.kIntakeRPM,  SparkBase.ControlType.kVelocity);
        //m_feederMotor.getClosedLoopController().setSetpoint(setpointRotationsPerSecondFeeder,  SparkBase.ControlType.kVelocity); //uncomment to add feeder motor back in
        }
    ).withName("Intake");
  }

  public Command speedupCommand() {
    return 
      // Run the shooter flywheel at the desired setpoint using feedforward and feedback
      run(
          () -> {
            m_shooterMotor.getClosedLoopController().setSetpoint( ShooterConstants.kTargetRPM, SparkBase.ControlType.kVelocity);
          }
      )
      .until( () -> m_shooterMotor.getClosedLoopController().isAtSetpoint())
      .withName("SpeedUp");
  }

  @Override
  public void periodic() {
    // Log values for AdvantageScope real time
    Logger.recordOutput("Shooter/setpoint", m_shooterMotor.getClosedLoopController().getSetpoint());
    Logger.recordOutput("Shooter/velocity", m_shooterEncoder.getVelocity());
    Logger.recordOutput("Shooter/appliedOutput", m_shooterMotor.getAppliedOutput());
    Logger.recordOutput("Shooter/ballInShooter", ballInShooter());
    
    // Publish REV 2m distance sensor value if present
    if (m_rev2m != null) {
   
      double rangeMM = m_rev2m.getRange(Rev2mDistanceSensor.Unit.kMillimeters);

      m_ballInShooter = (rangeMM < 300);
      if (m_ballCountDebounce <= 0) {
        if (m_ballInShooter && !m_lastBallInShooter) {
          m_ballCount++;
          m_ballCountDebounce = 3;
        }
      } else {
        m_ballCountDebounce--;
      }
      m_lastBallInShooter = m_ballInShooter;

      SmartDashboard.putNumber("Shooter/Rev2mDistanceMM", rangeMM);
      Logger.recordOutput("Shooter/distance", rangeMM);
    } else {
      SmartDashboard.putNumber("Shooter/Rev2mDistanceMeters", Double.NaN);
    }
      SmartDashboard.putBoolean("Shooter/fault", m_faultPresent);
      SmartDashboard.putNumber("Shooter/ballCount", m_ballCount);

    
    
  }

  /*
   * Simulate the shooter with a flywheel simulation and REV simulation helpers. This is used to test
   * the shooter code in simulation without needing a physical robot, and also to verify that the
   * flywheel model is reasonable.
   */

  @Override
  public void simulationPeriodic() {
    if (m_flywheelSim == null || m_shooterMotor == null) return;
    
    // Measure the actual time that has passed since the last simulation update,
    // and use that as the timestep for advancing the flywheel simulation. This is
    // more accurate than assuming a constant timestep (e.g. 20ms),
    // especially if the simulation is running slowly.
    
    double now = Timer.getFPGATimestamp();
    double dt = now - m_lastSimTimestamp;  // Advance the flywheel sim and update REV encoder sim helpers when running in simulation.
    m_lastSimTimestamp = now;

    // Estimate the battery voltage from the simulation
    // value is adjusted to account for the current being drawn by the simulated motors.

    double busVoltage = RobotController.getBatteryVoltage();
    double motorVoltage = busVoltage * m_shooterMotor.getAppliedOutput(); 

    if ( Math.abs(motorVoltage) < simStaticV) {
      motorVoltage = 0.0; // simulate static friction
    } else {
      motorVoltage = motorVoltage - Math.signum(motorVoltage) * simStaticV;  // simulate losses from friction
    }

    try {
      m_flywheelSim.setInputVoltage( motorVoltage);
      m_flywheelSim.update(dt);
      double angVelRPM = m_flywheelSim.getAngularVelocityRPM();
      m_simVelocity = angVelRPM; // convert to RPM
    } catch (Throwable ignored) {
      // If the flywheel sim fails for any reason, just skip the sim update and use the last velocity value.
      m_simVelocity = motorVoltage / ShooterConstants.kVVoltSecondsPerRotation * 60.0; // fallback to a simple motor model (ignore inertia) if the flywheel sim fails
    }

    // this updates the simulated encoder too
    m_shooterSim.iterate(m_simVelocity, busVoltage, dt);

    // adjust the simulated battery voltage based on simulated current draw from the motor
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage( m_shooterMotor.getOutputCurrent()));
  }

  public int getBallCount() {
    return m_ballCount;
  }

  public void resetBallCount() {
    m_ballCount = 0;
    m_ballCountDebounce = 0;    
  }

  private void resetFault() {
    m_faultPresent = false;
  }
}
