package frc.robot.autos;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

// compares a pathplanner and bline pose log for the same path
// matches each bline pose to the closest pathplanner pose instead of going by time, since they
// dont go the same speed
public final class PoseComparison {
  // end of path limits
  public static final double kFinalPoseToleranceMeters = 0.2;
  public static final double kFinalHeadingToleranceDeg = 5.0;
  // limits for every point along the way
  public static final double kSampleToleranceMeters = 0.05;
  public static final double kSampleHeadingToleranceDeg = 3.0;

  public record Result(
      String name,
      double rmsPositionErrorMeters,
      double maxPositionErrorMeters,
      double rmsHeadingErrorDeg,
      double maxHeadingErrorDeg,
      double finalPositionErrorMeters,
      double finalHeadingErrorDeg,
      int samples) {

    public boolean finalPosePasses() {
      return finalPositionErrorMeters <= kFinalPoseToleranceMeters
          && finalHeadingErrorDeg <= kFinalHeadingToleranceDeg;
    }

    public boolean samplesPass() {
      return maxPositionErrorMeters <= kSampleToleranceMeters
          && maxHeadingErrorDeg <= kSampleHeadingToleranceDeg;
    }

    @Override
    public String toString() {
      return String.format(
          "%-26s samples=%4d  pos RMS=%.3f m max=%.3f m  heading RMS=%.2f° max=%.2f°  "
              + "final: %.3f m %.2f°  [final %s, per-sample %s]",
          name, samples, rmsPositionErrorMeters, maxPositionErrorMeters, rmsHeadingErrorDeg,
          maxHeadingErrorDeg, finalPositionErrorMeters, finalHeadingErrorDeg,
          finalPosePasses() ? "PASS" : "FAIL", samplesPass() ? "PASS" : "FAIL");
    }
  }

  private PoseComparison() {}

  // matches each pose in actual to the closest pose in reference
  public static Result compare(String name, List<Pose2d> reference, List<Pose2d> actual) {
    double sumSqPos = 0, maxPos = 0, sumSqHeading = 0, maxHeading = 0;
    for (Pose2d pose : actual) {
      Pose2d nearest = nearest(reference, pose);
      double pos = nearest.getTranslation().getDistance(pose.getTranslation());
      double heading = headingErrorDeg(nearest, pose);
      sumSqPos += pos * pos;
      sumSqHeading += heading * heading;
      maxPos = Math.max(maxPos, pos);
      maxHeading = Math.max(maxHeading, heading);
    }
    int n = Math.max(1, actual.size());
    Pose2d refEnd = reference.get(reference.size() - 1);
    Pose2d actualEnd = actual.get(actual.size() - 1);
    return new Result(
        name,
        Math.sqrt(sumSqPos / n),
        maxPos,
        Math.sqrt(sumSqHeading / n),
        maxHeading,
        refEnd.getTranslation().getDistance(actualEnd.getTranslation()),
        headingErrorDeg(refEnd, actualEnd),
        actual.size());
  }

  public static double headingErrorDeg(Pose2d a, Pose2d b) {
    return Math.abs(a.getRotation().minus(b.getRotation()).getDegrees());
  }

  private static Pose2d nearest(List<Pose2d> poses, Pose2d target) {
    Pose2d best = poses.get(0);
    double bestDist = Double.MAX_VALUE;
    for (Pose2d pose : poses) {
      double d = pose.getTranslation().getDistance(target.getTranslation());
      if (d < bestDist) {
        bestDist = d;
        best = pose;
      }
    }
    return best;
  }

  // writes time_s,x_m,y_m,heading_deg so you can plot it or use tools/compare_pose_logs.py
  public static void writeCsv(Path file, List<Pose2d> poses, double dtSeconds) throws IOException {
    Files.createDirectories(file.getParent());
    try (PrintWriter out = new PrintWriter(Files.newBufferedWriter(file))) {
      out.println("time_s,x_m,y_m,heading_deg");
      for (int i = 0; i < poses.size(); i++) {
        Pose2d p = poses.get(i);
        out.printf("%.3f,%.4f,%.4f,%.3f%n", i * dtSeconds, p.getX(), p.getY(), p.getRotation().getDegrees());
      }
    }
  }
}
