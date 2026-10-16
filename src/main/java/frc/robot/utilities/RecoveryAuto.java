package frc.robot.utilities;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Loads a PathPlanner .auto file the same way PathPlannerAuto does, except every path in it runs
 * through CommandSwerveDrivetrain.followPathWithRecovery, so the robot pathfinds back on course if
 * it gets knocked off a path. Any auto whose name ends in " Recovery" is loaded this way, so the
 * recovery autos stay editable in the PathPlanner GUI like normal autos.
 */
public class RecoveryAuto {
  public static final String kSuffix = " Recovery";

  public static boolean isRecoveryAuto(String autoName) {
    return autoName.endsWith(kSuffix);
  }

  public static PathPlannerAuto build(String autoName, CommandSwerveDrivetrain drivetrain) throws Exception {
    JsonNode json = new ObjectMapper().readTree(
        new File(Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto"));

    List<PathPlannerPath> paths = new ArrayList<>();
    Command cmd = commandFromJson(json.get("command"), drivetrain, paths);

    // Same starting pose PathPlannerAuto uses: start of the first path in the auto
    Pose2d startingPose = null;
    if (!paths.isEmpty()) {
      PathPlannerPath path0 = paths.get(0);
      startingPose = new Pose2d(path0.getPoint(0).position, path0.getIdealStartingState().rotation());
    }
    if (json.path("resetOdom").asBoolean(false) && startingPose != null) {
      cmd = Commands.sequence(AutoBuilder.resetOdom(startingPose), cmd);
    }

    PathPlannerAuto auto = new PathPlannerAuto(cmd, startingPose);
    auto.setName(autoName);
    return auto;
  }

  // Mirrors PathPlanner's CommandUtil.commandFromJson, swapping in followPathWithRecovery for paths
  private static Command commandFromJson(JsonNode commandJson, CommandSwerveDrivetrain drivetrain,
      List<PathPlannerPath> paths) throws Exception {
    JsonNode data = commandJson.get("data");

    switch (commandJson.get("type").asText()) {
      case "wait":
        return Commands.waitSeconds(data.get("waitTime").asDouble());
      case "named":
        return NamedCommands.getCommand(data.get("name").asText());
      case "path":
        PathPlannerPath path = PathPlannerPath.fromPathFile(data.get("pathName").asText());
        paths.add(path);
        return drivetrain.followPathWithRecovery(path);
      case "sequential":
        return Commands.sequence(childCommands(data, drivetrain, paths));
      case "parallel":
        return Commands.parallel(childCommands(data, drivetrain, paths));
      case "race":
        return Commands.race(childCommands(data, drivetrain, paths));
      case "deadline":
        Command[] children = childCommands(data, drivetrain, paths);
        if (children.length == 0) {
          return Commands.none();
        }
        Command[] others = new Command[children.length - 1];
        System.arraycopy(children, 1, others, 0, others.length);
        return Commands.deadline(children[0], others);
      default:
        return Commands.none();
    }
  }

  private static Command[] childCommands(JsonNode data, CommandSwerveDrivetrain drivetrain,
      List<PathPlannerPath> paths) throws Exception {
    List<Command> commands = new ArrayList<>();
    for (JsonNode child : data.get("commands")) {
      commands.add(commandFromJson(child, drivetrain, paths));
    }
    return commands.toArray(new Command[0]);
  }
}
