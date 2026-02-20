// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StorageConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.spark.SparkBase;

@Logged
public class Storage extends SubsystemBase {
   private final SparkMaxConfig storageMotorConfig = new SparkMaxConfig();
  @NotLogged // We'll log a more meaningful boolean instead
 // private final DigitalInput m_ballSensor = new DigitalInput(StorageConstants.kBallSensorPort);
 private SparkMax m_storageMotor;
  private RelativeEncoder m_storageEncoder;
  // Expose trigger from subsystem to improve readability and ease
  // inter-subsystem communications
  /** Whether the ball storage is full. */
  //@Logged(name = "Has Cargo")
  //@SuppressWarnings("checkstyle:MemberName")
  //public final Trigger hasCargo = new Trigger(m_ballSensor::get);

  /** Create a new Storage subsystem. */
  public Storage() {
         SparkMaxConfig globalConfig = new SparkMaxConfig();
   
    globalConfig
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    globalConfig.closedLoop.pid(StorageConstants.kP, 0.0, StorageConstants.kD);
    globalConfig.closedLoop.allowedClosedLoopError(StorageConstants.kShooterToleranceRPS,ClosedLoopSlot.kSlot0);
    globalConfig.closedLoop.feedForward.kS(StorageConstants.kSVolts);
    globalConfig.closedLoop.feedForward.kV(StorageConstants.kVVoltSecondsPerRotation);
       
    globalConfig.encoder.countsPerRevolution(StorageConstants.kEncoderCPR);
    globalConfig.encoder.positionConversionFactor((float) 1.0 / 8.4); // to get revolutions of motor per pulse
    globalConfig.encoder.velocityConversionFactor((float) 1.0 / 60.0); // revs per second

     storageMotorConfig
        .apply(globalConfig)
        .inverted(true);
          m_storageMotor = new SparkMax(DriveConstants.kFeederMotorID, MotorType.kBrushed);
          try { m_storageEncoder = m_storageMotor.getEncoder(); } catch (Throwable ignored) {}
    // Set default command to turn off the storage motor and then idle
    setDefaultCommand(runOnce(m_storageMotor::disable).andThen(run(() -> {})).withName("Idle"));
  }

  /** Returns a command that runs the storage motor indefinitely. */
  public Command runCommand(boolean moveDirection) {
    if (moveDirection) {
      return run(() -> m_storageMotor.set(1)).withName("run");
    } else {
       return run(() -> m_storageMotor.set(-1)).withName("run");
    }
     //
    
  }


}
