package frc.robot.commands;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

import java.util.List;
import java.util.function.Supplier;

/**
 * AutoCommands: simple trajectory follower using LTVUnicycleController and wheel velocity control.
 * This intentionally does NOT use RamseteCommand or voltage control; it samples the trajectory
 * and uses the LTV controller to compute chassis speeds, converts to wheel speeds, and sets
 * closed-loop wheel velocity setpoints on the Drive subsystem.
 */
public final class AutoCommands {
  private AutoCommands() {}

  /**
   * Build a command that drives forward approximately 2 meters from the current pose.
   * The command resets odometry on init and follows a simple straight-line trajectory.
   */
  public static Command followStraight2m(Drive drive) {
    TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.kMaxSpeedMetersPerSecond / 2.0,
        DriveConstants.kMaxSpeedMetersPerSecond / 3.0)
        .setKinematics(drive.getKinematics());

    Supplier<Trajectory> supplier = () -> {
      Pose2d start = drive.getPose();
      Pose2d end = new Pose2d(start.getX() + 2.0, start.getY(), start.getRotation());
      return TrajectoryGenerator.generateTrajectory(start, List.of(), end, config);
    };

    return followTrajectory(drive, supplier);
  }

  /**
   * Drive to a fixed pose (3,4,0) passing through waypoint (3,1,0).
   * Uses the same LTVUnicycleController -> chassis speeds -> Drive.setChassisSpeeds pipeline.
   */
  public static Command followToPoseViaWaypoint(Drive drive) {
    TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.kMaxSpeedMetersPerSecond / 2.0,
        DriveConstants.kMaxSpeedMetersPerSecond / 3.0)
        .setKinematics(drive.getKinematics());

    Supplier<Trajectory> supplier = () -> {
      Pose2d start = drive.getPose();
      Translation2d waypoint = new Translation2d(3.0, 1.0);
      Pose2d goal = new Pose2d(3.0, 4.0, new Rotation2d(0.0));
      return TrajectoryGenerator.generateTrajectory(start, List.of(waypoint), goal, config);
    };

    return followTrajectory(drive, supplier);
  }

  /**
   * Execute a trajectory using LTVUnicycleController and command the drive with chassis speeds.
   * The trajectory is supplied lazily (supplier invoked during initialize) so it can be built
   * from the robot's pose after any odometry reset.
   */
  private static Command followTrajectory(Drive drive, Supplier<Trajectory> trajSupplier) {
    final Trajectory[] trajHolder = new Trajectory[1];
    final Timer timer = new Timer();
    final LTVUnicycleController controller = new LTVUnicycleController(0.02);

    return new FunctionalCommand(
        // initialize
        () -> {
          drive.resetOdometry();
          trajHolder[0] = trajSupplier.get();
          timer.reset();
          timer.start();
        },
        // execute
        () -> {
          Trajectory traj = trajHolder[0];
          if (traj == null) {
            return;
          }
          double t = timer.get();
          if (t > traj.getTotalTimeSeconds()) {
            return;
          }
          Trajectory.State desired = traj.sample(t);
          ChassisSpeeds chassisSpeeds = controller.calculate(drive.getPose(), desired);
          drive.setChassisSpeeds(chassisSpeeds);
        },
        // end
        interrupted -> {
          timer.stop();
          drive.stop();
        },
        // isFinished
        () -> {
          Trajectory traj = trajHolder[0];
          return traj != null && timer.hasElapsed(traj.getTotalTimeSeconds());
        },
        drive);
}

}
