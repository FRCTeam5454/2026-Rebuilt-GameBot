package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.FieldConstants;
import frc.robot.Constants;
import frc.robot.RobotState;
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
     * @param drivebase    the swerve drivetrain (only used here as the deferred command's
     *                     requirement; the actual pose comes from {@link RobotState}, which stays
     *                     vision-corrected, instead of the drivetrain's own pose)
     */
    public static Command alignToClimb(boolean isAlignRight, CommandSwerveDrivetrain drivebase) {
        return Commands.defer(() -> {
            PathConstraints constraints = new PathConstraints(3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

            Pose2d startPose = FieldConstants.flipIfRed(RobotState.getInstance().getEstimatedPose());

            Pose2d climbPose = new Pose2d(
                isAlignRight ? flipLR(Constants.ClimbConstants.AutoAlign.ySetpoint) : Constants.ClimbConstants.AutoAlign.ySetpoint,
                Constants.ClimbConstants.AutoAlign.xSetpoint,
                startPose.getRotation()
            );

            return AutoBuilder.pathfindToPose(climbPose, constraints, 0.0);
        }, Set.of(drivebase));
    }
}
