// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.epilogue.Logged;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@Logged(name = "Rapid React Command Robot Container")
public class RapidReactCommandBot {
  // The robot's subsystems
  private final Drive m_drive = new Drive();
  private final Intake m_intake = new Intake();
  private final Storage m_storage = new Storage();
 private final Shooter m_shooter = new Shooter();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /**
   * Use this method to define bindings between conditions and commands. These are useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>Should be called in the robot class constructor.
   *
   * <p>Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {
    // Automatically run the storage motor whenever the ball storage is not full,
    // and turn it off whenever it fills. Uses subsystem-hosted trigger to
    // improve readability and make inter-subsystem communication easier.
   // m_storage.hasCargo.whileFalse(m_storage.runCommand());

    // Automatically disable and retract the intake whenever the ball storage is full.
//    m_storage.hasCargo.onTrue(m_intake.retractCommand());

    // Control the drive with split-stick arcade controls
    m_drive.setDefaultCommand(
        
     m_drive.arcadeDriveCommand(  () -> -m_driverController.getLeftY() / 1.75 , () -> -m_driverController.getRightX() / 2, false)
     //m_drive.arcadeDriveCommand(  () -> {if (-m_driverController.getLeftY() >=0) {return Math.sqrt(-m_driverController.getLeftY() / 1.75);} else { return -1 * Math.sqrt(m_driverController.getLeftY() / 1.75);}} , () -> {if (-m_driverController.getLeftY() >=0) {return Math.sqrt(-m_driverController.getRightX() / 1.1);} else { return -1 * Math.sqrt(m_driverController.getRightX() / 1.1);}}) //square rooted traditional (I think this is a bad idea, but wesley wants it, so...)
          //  m_drive.arcadeDriveCommand(() -> (m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()), () -> -m_driverController.getLeftX()) // Trigger to go foreward or backward
   // m_drive.tankDriveCommand( () -> -m_driverController.getLeftY() , () -> -m_driverController.getRightY()  ) //Tank drive, sometimes divide by 3
           // m_drive.arcadeDriveCommand(  () -> -m_driverController.getLeftY() * Math.abs(m_driverController.getLeftY()),
            // () -> -m_driverController.getRightX() * Math.abs(m_driverController.getRightX()))  //squared input traditional

   
  
   
   
   );



    // Deploy the intake with the X button
  //  m_driverController.x().onTrue(m_intake.intakeCommand());
    // Retract the intake with the Y button
  //  m_driverController.y().onTrue(m_intake.retractCommand());

    // Fire the shooter with the A button
   
     m_driverController
        .a()
        .onTrue(
            parallel(
                    m_shooter.shootCommand(ShooterConstants.kShooterTargetRPS),
                    m_storage.runCommand())
                // Since we composed this inline we should give it a name
                .withName("Shoot"));


  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Drive forward for 2 meters at half speed with a 3 second timeout
    return m_drive
    //     .turnToAngleCommand(190);}  
   //     .driveDistanceCommand(AutoConstants.kDriveDistanceMeters, AutoConstants.kDriveSpeed)
   //     .withTimeout(AutoConstants.kTimeoutSeconds);
   .forwardBackCommand(1, 1, 2, 2)
   .withTimeout(AutoConstants.kTimeoutSeconds);}

  public Drive getDrive() {
    return m_drive;
  }

  public void resetDrive() {
    m_drive.resetOdometry();
  }

}
