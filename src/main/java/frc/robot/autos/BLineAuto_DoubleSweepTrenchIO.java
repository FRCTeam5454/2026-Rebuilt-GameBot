package frc.robot.autos;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

/*
 * bline version of the "Double sweep TrenchIO" pathplanner auto, the pathplanner one is still there
 *
 * same order as the .auto:
 *   expandIntake -> out -> Back -> shotLookUpW5 -> expandIntake -> out2 -> Back2 -> shotLookUpW5  copilot just knows what I'm boutta say and finishes my sentences I guess I think like an AI
 * paths are in deploy/autos/paths/DoubleSweepTrenchIO_*.json, made from the pathplanner paths
 * with tools/pp_to_bline.py (rerun that if the pathplanner paths change)
 *
 * notes:
 * - bline only does straight lines so the curves are chopped into points every 0.25m,
 *   all the pathplanner anchors and rotation targets are kept
 * - same 4.75 m/s, 7.7 m/s^2, 720 deg/s, 596 deg/s^2 as pathplanner
 * - bline doesnt slow down for curves so the script caps speed in the curves like pathplanner
 *   does, without that it went up to 1.1m wide
 * - no event markers in these paths so nothing to map there
 * - red alliance flips the same way pathplanner does
 * - heading lags pathplanner mid path (up to ~24 deg) since bline has no rotation feedforward,
 *   it catches up by the end of each path
 *  following other teams style this seemed to be how BLine was intended to be used, so we can try it and see if it works well enough
 * This is a lot of math :(
 */
public final class BLineAuto_DoubleSweepTrenchIO {
  // name in the auto chooser
  public static final String kName = "BLine Double sweep TrenchIO";

  // bline path files in the order they run
  public static final String[] kPathNames = {
    "DoubleSweepTrenchIO_out",
    "DoubleSweepTrenchIO_back",
    "DoubleSweepTrenchIO_out2",
    "DoubleSweepTrenchIO_back2",
  };

  // same translation P as pathPlanDriveController
  public static final double kTranslationP = 3.0;
  // pathplanner uses 3/0/1 but that lagged ~32 deg and took 13.2s for the 4 paths, this is 10.5s
  // (not tested on the real robot yet, watch for it wobbling)
  public static final double kRotationP = 8.0;
  public static final double kRotationD = 0.5;
  // pathplanner doesnt have this one, using the bline default
  public static final double kCrossTrackP = 2.0;

  private BLineAuto_DoubleSweepTrenchIO() {}

  // build a new one every time auto starts
  // expandIntake/shotLookUpW5 are suppliers so the test can swap in fake commands
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

    // only reset pose on the first path (like resetOdom in pathplanner)
    Command out = builder.withPoseReset(resetPose).build(paths[0]);
    builder.withPoseReset(pose -> {});
    Command back = builder.build(paths[1]);
    Command out2 = builder.build(paths[2]);
    Command back2 = builder.build(paths[3]);

    Command auto = Commands.sequence(
        expandIntake.get(),
        out,
        back,
        shotLookUpW5.get(),
        expandIntake.get(),
        out2,
        back2,
        shotLookUpW5.get());
    auto.setName(kName);
    return auto;
  }

  // loads from deploy/autos/paths
  public static Path[] loadPaths() {
    Path[] paths = new Path[kPathNames.length];
    for (int i = 0; i < kPathNames.length; i++) {
      paths[i] = new Path(kPathNames[i]);
    }
    return paths;
  }
}
