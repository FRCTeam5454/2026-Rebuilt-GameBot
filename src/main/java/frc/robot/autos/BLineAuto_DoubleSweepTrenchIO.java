package frc.robot.autos;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.lib.BLine.Path.Waypoint;

/*
 * TEST VERSION: straight 2 m forward, then straight 2 m back to the start.
 * Same class/name as the double sweep so the auto chooser and RobotContainer don't change.
 *
 * - starts at Trench Start (same spot as the double sweep), facing 0 deg (+X, away from blue wall)
 * - paths are built in code, no json files needed
 * - heading stays at 0 the whole time so this only tests translation
 * - red alliance flips like before
 * - expandIntake/shotLookUpW5 are kept in the signature so RobotContainer still compiles, but not used
 */
public final class BLineAuto_DoubleSweepTrenchIO {
  // name in the auto chooser
  public static final String kName = "BLine Double sweep TrenchIO";

  public static final String[] kPathNames = {
    "StraightTest_out",
    "StraightTest_back",
  };

  // start pose = Trench Start from the pathplanner paths
  public static final Pose2d kStartPose = new Pose2d(4.3884, 0.4057, Rotation2d.kZero);
  public static final double kDistanceMeters = 2.0;

  // slower than the config defaults (4.5 m/s, 12 m/s^2) for a first test
  public static final double kMaxVelocity = 2.0;
  public static final double kMaxAccel = 3.0;

  public static final double kTranslationP = 3.0;
  public static final double kRotationP = 8.0;
  public static final double kRotationD = 0.5;
  public static final double kCrossTrackP = 2.0;

  private BLineAuto_DoubleSweepTrenchIO() {}

  // build a new one every time auto starts
  public static Command build(
      Subsystem drive,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
      Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer,
      Consumer<Pose2d> resetPose,
      Supplier<Command> expandIntake,
      Supplier<Command> shotLookUpW5) {
    Path[] paths = loadPaths();

    FollowPath.Builder builder = new FollowPath.Builder(
        drive,
        poseSupplier,
        robotRelativeSpeedsSupplier,
        robotRelativeSpeedsConsumer,
        new PIDController(kTranslationP, 0.0, 0.0),
        new PIDController(kRotationP, 0.0, kRotationD),
        new PIDController(kCrossTrackP, 0.0, 0.0))
        .withDefaultShouldFlip();

    // reset pose to the start on the first path only
    Command out = builder.withPoseReset(resetPose).build(paths[0]);
    builder.withPoseReset(pose -> {});
    Command back = builder.build(paths[1]);

    Command auto = Commands.sequence(out, back);
    auto.setName(kName);
    return auto;
  }

  // two straight lines, built in code
  public static Path[] loadPaths() {
    Pose2d endPose = new Pose2d(
        kStartPose.getX() + kDistanceMeters, kStartPose.getY(), kStartPose.getRotation());

    PathConstraints constraints = new PathConstraints()
        .setMaxVelocityMetersPerSec(kMaxVelocity)
        .setMaxAccelerationMetersPerSec2(kMaxAccel);

    Path out = new Path(constraints, new Waypoint(kStartPose), new Waypoint(endPose));
    Path back = new Path(constraints.copy(), new Waypoint(endPose), new Waypoint(kStartPose));
    return new Path[] {out, back};
  }
}