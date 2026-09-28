package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.FieldConstants;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Builds the pathfind-to-climb-pose command used by RobotContainer's climb sequences.
 *
 * <p>This used to be its own {@code Command} subclass that called
 * {@code CommandScheduler.getInstance().schedule(...)} from inside {@code initialize()} to kick
 * off {@link AutoBuilder#pathfindToPose}. That's a bug: the pathfind command requires the same
 * drivetrain subsystem this command already required, so scheduling it mid-sequence caused the
 * scheduler to cancel the entire outer {@code Commands.sequence(extend, align, retract)} -
 * dropping the {@code retract} step every time.
 *
 * <p>{@link Commands#defer} is the correct tool here: it builds the real command lazily, at the
 * moment this step of the sequence actually starts, using live pose/alliance data - and composes
 * normally with no manual scheduling required.
 */
public final class AutoAlignPose {
    private AutoAlignPose() {}

    private static double flipLR(double value) {
        return 8.07 - value; //distance from wall (top) of field translated to distance from wall (bottom) of field
    }

    /**
     * @param isAlignRight which side of the climb to align to
     * @param drivebase    the swerve drivetrain (the deferred command's requirement, and the
     *                     source of the vision-corrected pose)
     */
    public static Command alignToClimb(boolean isAlignRight, CommandSwerveDrivetrain drivebase) {
        return Commands.defer(() -> {
            PathConstraints constraints = new PathConstraints(3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

            // Setpoints are blue-alliance field coordinates: xSetpoint is distance from the
            // alliance wall, ySetpoint is distance across the field (flipLR mirrors it).
            Translation2d climbBlue = new Translation2d(
                Constants.ClimbConstants.AutoAlign.xSetpoint,
                isAlignRight ? flipLR(Constants.ClimbConstants.AutoAlign.ySetpoint) : Constants.ClimbConstants.AutoAlign.ySetpoint
            );
            // pathfindToPose doesn't flip for red, so flip the target ourselves
            Translation2d climbTranslation =
                FieldConstants.flipIfRed(new Pose2d(climbBlue, Rotation2d.kZero)).getTranslation();

            // Keep the robot's current heading - verify the heading you want at the climb
            Pose2d climbPose = new Pose2d(climbTranslation, drivebase.getPose2d().getRotation());

            return AutoBuilder.pathfindToPose(climbPose, constraints, 0.0);
        }, Set.of(drivebase));
    }
}
