// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1ID = 1;
    public static final int kLeftMotor2ID = 2;
    public static final int kRightMotor1ID = 3;
    public static final int kRightMotor2ID = 4;
    public static final int kShooterMotorID = 10;
    public static final int kFeederMotorID = 11;


    public static final int[] kLeftEncoderPorts = {0, 1};
    public static final int[] kRightEncoderPorts = {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;
    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These values MUST be determined either experimentally or theoretically for *your* robot's
    // drive. The SysId tool provides a convenient method for obtaining feedback and feedforward
    // values for your robot.
    public static final double kTurnP = 0.6;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerDegree = 0.8;
    public static final double kaVoltSecondsSquaredPerDegree = 0.15;

    // Distance between left and right wheels (track width) for Andymark 2024 Kitbot
    // Approximate standard value in meters; tune for your robot.
    public static final double kTrackwidthMeters = 0.69;

    // Drive motor velocity PID (used for internal SparkMax PID tuning)
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    // Feedforward for SparkMax internal velocity controller (units depend on controller config)
    public static final double kDriveFF = 0.0;
    // Gear reduction between motor and wheel (motor rotations per wheel rotation)
    // For example: an 8.46:1 reduction means 8.46 motor rotations = 1 wheel rotation
    public static final double kDriveGearReduction = 8.46;

  // Nominal free speed of the drive motor (NEO) in RPM. Used for simple sim.
  public static final double kMotorFreeSpeedRPM = 5676.0;

  // Derived maximum linear speed (meters per second) at the wheel assuming free
  // motor speed and the configured gear reduction. This is only a simple
  // theoretical value and should be tuned/verified for simulation fidelity.
  public static final double kMaxSpeedMetersPerSecond =
    (kMotorFreeSpeedRPM / kDriveGearReduction) / 60.0 * (Math.PI * kWheelDiameterMeters);
    // Slew-rate limiter rates (units: input units per second). These limit how
    // quickly joystick inputs can change. Tune to taste.
    public static final double kSlewRateForward = 1.0; // per second
    public static final double kSlewRateRotate = 1.0; // per second
    // Allowed range for slew-rate limiter configuration
    public static final double kSlewMin = 0.0;
    public static final double kSlewMax = 1.0;
  }

  public static final class ShooterConstants {
    public static final int[] kEncoderPorts = {4, 5};
    public static final boolean kEncoderReversed = false;
    public static final int kEncoderCPR = 8192;
    // Distance units will be rotations
    public static final double kEncoderDistancePerPulse = 1.0 / kEncoderCPR;

    public static final int kShooterMotorPort = 4;
    public static final int kFeederMotorPort = 5;

    public static final double kShooterFreeRPS = 5300.0 / 60.0;
    public static final double kShooterTargetRPS = 80.0;
    public static final double kShooterToleranceRPS = 0.05;
    public static final double kFeederTargetRPS = 30.0; //ai generated number, not permanant

    // These are not real PID gains, and will have to be tuned for your specific robot.
    public static final double kP = 0.07;
    public static final double kD = 0.01;

    // On a real robot the feedforward constants should be empirically determined; these are
    // reasonable guesses.
    public static final double kSVolts = 0.1; // 0.05;
    // Should have value 12V at free speed
    public static final double kVVoltSecondsPerRotation = 12.0 / kShooterFreeRPS;

    public static final double kFeederSpeed = 0.5;
  }

  public static final class IntakeConstants {
    public static final int kMotorPort = 6;
    public static final int[] kSolenoidPorts = {2, 3};
  }

  public static final class StorageConstants {
    public static final int kMotorPort = 7;
    public static final int kBallSensorPort = 6;
    public static final int kEncoderCPR = 8192;
    public static final double kP = 0.07;
    public static final double kD = 0.01;
    public static final double kShooterToleranceRPS = 0.5;
    public static final double kSVolts = 0.1; // 0.05;
    public static final double kVVoltSecondsPerRotation = 12.0 / (5300.0 / 60.0);  
    public static final int kstorageCANID = 12;
  }

  public static final class AutoConstants {
    public static final double kTimeoutSeconds = 6;
    public static final double kDriveDistanceMeters = 2;
    public static final double kDriveSpeed = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}
