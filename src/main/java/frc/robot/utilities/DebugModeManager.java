// Copyright (c) 2025-2026 Mavericks
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import lombok.Getter;
import lombok.Setter;

/**
 * Centralized debug mode manager for the robot.
 *
 * <p>This class provides a single point of control for debug mode across the entire robot
 * codebase. When debug mode is enabled:
 * <ul>
 *   <li>Debug-level logs are recorded and displayed
 *   <li>Additional subsystem diagnostics are logged
 *   <li>Verbose telemetry is sent to dashboards
 *   <li>Performance metrics are tracked
 * </ul>
 *
 * <p>Usage:
 *
 * <pre>{@code
 * // Enable debug mode
 * DebugModeManager.setDebugMode(true);
 *
 * // Check if debug mode is enabled
 * if (DebugModeManager.isDebugMode()) {
 *   // Perform debug operations
 * }
 *
 * // Log debug information
 * if (DebugModeManager.isDebugMode()) {
 *   LoggingUtil.logDebug("Subsystem/Value", value);
 * }
 * }</pre>
 */
public class DebugModeManager {
  @Getter @Setter private static boolean debugMode = false;
  @Getter @Setter private static boolean logToFile = true;
  @Getter @Setter private static boolean logToNetworkTables = true;
  @Getter @Setter private static boolean logToSmartDashboard = true;

  private static final String DEBUG_KEY = "Debug/DebugMode";
  private static final String LOG_FILE_KEY = "Debug/LogToFile";
  private static final String LOG_NT_KEY = "Debug/LogToNetworkTables";
  private static final String LOG_SD_KEY = "Debug/LogToSmartDashboard";

  /**
   * Initialize the debug mode manager.
   *
   * <p>This should be called once during robot initialization to set up the SmartDashboard
   * widgets and configure logging.
   */
  public static void initialize() {
    // Add SmartDashboard controls
    SmartDashboard.putBoolean(DEBUG_KEY, debugMode);
    SmartDashboard.putBoolean(LOG_FILE_KEY, logToFile);
    SmartDashboard.putBoolean(LOG_NT_KEY, logToNetworkTables);
    SmartDashboard.putBoolean(LOG_SD_KEY, logToSmartDashboard);

    LoggingUtil.logString("DebugMode/Initialized", "Debug mode manager initialized");
  }

  /**
   * Update debug mode from SmartDashboard values.
   *
   * <p>This should be called periodically (e.g., in robotPeriodic()) to sync SmartDashboard
   * values with the debug mode state.
   */
  public static void update() {
    // Update from SmartDashboard
    debugMode = SmartDashboard.getBoolean(DEBUG_KEY, debugMode);
    logToFile = SmartDashboard.getBoolean(LOG_FILE_KEY, logToFile);
    logToNetworkTables = SmartDashboard.getBoolean(LOG_NT_KEY, logToNetworkTables);
    logToSmartDashboard = SmartDashboard.getBoolean(LOG_SD_KEY, logToSmartDashboard);

    // Update LoggingUtil with current settings
    LoggingUtil.setDebugMode(debugMode);
    LoggingUtil.setSmartDashboardEnabled(logToSmartDashboard);

    // Push current state to SmartDashboard
    SmartDashboard.putBoolean(DEBUG_KEY, debugMode);
  }

  /**
   * Enable or disable debug mode and sync with logging system.
   *
   * @param enabled True to enable debug mode, false to disable
   */
  public static void setDebugModeAndSync(boolean enabled) {
    debugMode = enabled;
    LoggingUtil.setDebugMode(enabled);
    SmartDashboard.putBoolean(DEBUG_KEY, enabled);

    if (enabled) {
      LoggingUtil.logString("Debug/Status", "Debug mode ENABLED");
    } else {
      LoggingUtil.logString("Debug/Status", "Debug mode DISABLED");
    }
  }

  /**
   * Check if debug mode is currently enabled.
   *
   * @return True if debug mode is enabled, false otherwise
   */
  public static boolean isDebugEnabled() {
    return debugMode;
  }

  /**
   * Log debug information about a subsystem.
   *
   * @param subsystemName The name of the subsystem
   * @param message The debug message
   */
  public static void logDebugMessage(String subsystemName, String message) {
    if (debugMode) {
      LoggingUtil.logString("Debug/" + subsystemName, message);
    }
  }

  /**
   * Toggle debug mode.
   *
   * @return The new state of debug mode
   */
  public static boolean toggleDebugMode() {
    setDebugModeAndSync(!debugMode);
    return debugMode;
  }

  /**
   * Print debug information to console.
   *
   * @param tag The tag/category for this message
   * @param message The message to print
   */
  public static void printDebug(String tag, String message) {
    if (debugMode) {
      System.out.println("[DEBUG] " + tag + ": " + message);
      LoggingUtil.logDebug("Console/" + tag, message);
    }
  }
}
