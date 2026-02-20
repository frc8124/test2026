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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

@Logged
public class Shooter extends SubsystemBase {

  private SparkMax m_shooterMotor;
  private SparkMax m_feederMotor;


private RelativeEncoder m_shooterEncoder;
  // internal simulation state (rotations)
  private double m_simShooterPosition = 0.0;

  

     private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
     private final SparkMaxConfig feederMotorConfig = new SparkMaxConfig(); //uncomment to add feeder motor back in
     

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

     shooterMotorConfig
        .apply(globalConfig)
        .inverted(true);


    feederMotorConfig
        .apply(globalConfig)
        .inverted(false);
 // Initialize feeder motor

  m_shooterMotor = new SparkMax(DriveConstants.kShooterMotorID, MotorType.kBrushed);
  m_feederMotor = new SparkMax(DriveConstants.kFeederMotorID, MotorType.kBrushed);
 //   m_shooterFeedback.setTolerance(ShooterConstants.kShooterToleranceRPS);
   // m_shooterEncoder.setDistancePerPulse(ShooterConstants.kEncoderDistancePerPulse);
  m_shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  
  m_shooterMotor.set(0);

  // Initialize shooter encoder wrapper from the motor controller
  try { m_shooterEncoder = m_shooterMotor.getEncoder(); } catch (Throwable ignored) {}
  m_feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); //uncomment to add feeder motor back in
    // Set default command to turn off both the shooter and feeder motors, and then idle
    setDefaultCommand(
        runOnce(
                () -> {
                  this.setSetpoint(0.0);
                  m_shooterMotor.set(0); // disable();
                  m_feederMotor.set(0); // disable();
                })
            .withName("Idle"));
  }

  /**
   * Returns a command to shoot the balls currently stored in the robot. Spins the shooter flywheel
   * up to the specified setpoint, and then runs the feeder motor.
   *
   * @param setpointRotationsPerSecond The desired shooter velocity
   * @param setpointRotationsPerSecondFeeder The desired feeder velocity
   */
  public Command shootCommand(double setpointRotationsPerSecond, double setpointRotationsPerSecondFeeder) {
    return // parallel(
            // Run the shooter flywheel at the desired setpoint using feedforward and feedback
            run(
                () -> {
                 this.setSetpoint(setpointRotationsPerSecond);
                 this.setSetpoint(setpointRotationsPerSecondFeeder);
                 m_shooterMotor.getClosedLoopController().setSetpoint(setpointRotationsPerSecond,  SparkBase.ControlType.kVelocity);
                m_feederMotor.getClosedLoopController().setSetpoint(setpointRotationsPerSecondFeeder,  SparkBase.ControlType.kVelocity); //uncomment to add feeder motor back in
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
    } catch (Throwable t) {
      // ignore errors in dashboard publishing
    }
  }

 

}
