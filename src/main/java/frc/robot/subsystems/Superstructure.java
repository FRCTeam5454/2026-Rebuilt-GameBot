// Copyright (c) 2025-2026 Mavericks
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utilities.LoggingUtil;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;

/**
 * Abstract base class for Superstructure implementations.
 *
 * <p>A Superstructure coordinates multiple subsystems that work together to accomplish a higher-level
 * goal (e.g., Shooter + Turret + Hood, or Intake + Indexer + Hopper). This base class provides:
 *
 * <ul>
 *   <li>Centralized periodic management of related subsystems
 *   <li>State machine framework for complex multi-subsystem actions
 *   <li>Unified logging and diagnostics
 *   <li>Simplified integration with RobotContainer
 * </ul>
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * public class ShooterSuperstructure extends Superstructure {
 *   private final Shooter shooter;
 *   private final Turret turret;
 *   private final Hood hood;
 *
 *   public ShooterSuperstructure(Shooter shooter, Turret turret, Hood hood) {
 *     super("ShooterSuperstructure");
 *     this.shooter = shooter;
 *     this.turret = turret;
 *     this.hood = hood;
 *
 *     registerSubsystems(shooter, turret, hood);
 *   }
 *
 *   @Override
 *   public void periodic() {
 *     super.periodic(); // Calls periodic on all registered subsystems
 *     // Coordinate subsystems here
 *   }
 * }
 * }</pre>
 */
public abstract class Superstructure extends Subsystem {
  @Getter private final String name;
  private final List<Subsystem> managedSubsystems = new ArrayList<>();
  private boolean enabled = true;

  /**
   * Create a new Superstructure with the given name.
   *
   * @param name The name of this superstructure
   */
  protected Superstructure(String name) {
    this.name = name;
    setName(name);
  }

  /**
   * Register subsystems to be managed by this Superstructure.
   *
   * <p>Registered subsystems will have their periodic() method called automatically, and their
   * telemetry will be logged under this Superstructure's namespace.
   *
   * @param subsystems The subsystems to manage
   */
  protected void registerSubsystems(Subsystem... subsystems) {
    for (Subsystem subsystem : subsystems) {
      if (subsystem != null) {
        managedSubsystems.add(subsystem);
      }
    }
    LoggingUtil.logString(
        name + "/Status", "Registered " + managedSubsystems.size() + " subsystems");
  }

  /**
   * Enable or disable this Superstructure.
   *
   * <p>When disabled, the periodic() method will still be called, but subsystem updates should be
   * skipped or idled.
   *
   * @param enabled True to enable, false to disable
   */
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    LoggingUtil.logBoolean(name + "/Enabled", enabled);
  }

  /**
   * Check if this Superstructure is enabled.
   *
   * @return True if enabled, false otherwise
   */
  public boolean isEnabled() {
    return enabled;
  }

  /**
   * Get all managed subsystems.
   *
   * @return List of managed subsystems
   */
  public List<Subsystem> getManagedSubsystems() {
    return new ArrayList<>(managedSubsystems);
  }

  /**
   * Called periodically by the CommandScheduler. This method calls periodic() on all registered
   * subsystems and then calls the subclass's periodicImpl() method.
   *
   * <p>Override periodicImpl() in subclasses, not this method.
   */
  @Override
  public final void periodic() {
    // Call periodic on all managed subsystems
    for (Subsystem subsystem : managedSubsystems) {
      subsystem.periodic();
    }

    // Call subclass-specific periodic implementation
    if (enabled) {
      periodicImpl();
    } else {
      idleImpl();
    }

    // Log superstructure status
    logStatus();
  }

  /**
   * Subclass-specific periodic implementation.
   *
   * <p>Override this method in subclasses to implement periodic logic. This is called after all
   * managed subsystems have had their periodic() method called.
   */
  protected abstract void periodicImpl();

  /**
   * Called when this Superstructure is disabled.
   *
   * <p>Override this method to implement idle/safe behavior when the Superstructure is disabled.
   * Default implementation does nothing.
   */
  protected void idleImpl() {
    // Default: do nothing when idle
  }

  /**
   * Log the status of this Superstructure.
   *
   * <p>Override this method to add custom telemetry. Remember to call super.logStatus() to
   * maintain base telemetry.
   */
  protected void logStatus() {
    LoggingUtil.logBoolean(name + "/Enabled", enabled);
    LoggingUtil.logNumber(name + "/NumSubsystems", managedSubsystems.size());
  }

  /**
   * Safely execute an action on a managed subsystem.
   *
   * <p>This method provides a consistent way to interact with subsystems managed by this
   * Superstructure, with built-in error handling and logging.
   *
   * @param subsystemName The name of the subsystem
   * @param action The action to perform
   * @return True if the action completed successfully, false otherwise
   */
  protected boolean safeExecute(String subsystemName, Runnable action) {
    try {
      if (!enabled) {
        LoggingUtil.logDebug(name + "/" + subsystemName, "Attempted action on disabled superstructure");
        return false;
      }
      action.run();
      return true;
    } catch (Exception e) {
      LoggingUtil.logException(name + "/" + subsystemName + "/Error", e);
      return false;
    }
  }
}
