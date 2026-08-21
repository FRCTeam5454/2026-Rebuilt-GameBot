// Copyright (c) 2025-2026  Mavericks
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized logging utility for the robot.
 *
 * <p>This class provides a unified interface for logging to both AdvantageKit and SmartDashboard,
 * supporting multiple log levels and debug mode. Use this instead of calling Logger or
 * SmartDashboard directly throughout the codebase.
 *
 * <p>Features:
 * <ul>
 *   <li>Unified logging interface for both AdvantageKit Logger and SmartDashboard
 *   <li>Support for log levels: DEBUG, INFO, WARN, ERROR
 *   <li>Debug mode toggle for verbose logging
 *   <li>Namespace-based organization
 *   <li>Easy migration path from scattered logging calls
 * </ul>
 *
 * <p>Usage:
 *
 * <pre>{@code
 * // Log to both AdvantageKit and SmartDashboard
 * LoggingUtil.logNumber("Subsystem/Speed", speed);
 * LoggingUtil.logBoolean("Subsystem/IsActive", isActive);
 *
 * // Debug logging (only logged if debug mode is enabled)
 * LoggingUtil.logDebug("Subsystem/DetailValue", detailValue);
 *
 * // Log with namespace
 * LoggingUtil.getInstance("Shooter").logNumber("Velocity", velocity);
 * }</pre>
 */
public class LoggingUtil {
  private static LoggingUtil instance;
  private static boolean debugMode = false;
  private static boolean advantageKitEnabled = true;
  private static boolean smartDashboardEnabled = true;

  private final String namespace;

  // Private constructor for singleton pattern
  private LoggingUtil() {
    this("");
  }

  // Constructor with namespace
  private LoggingUtil(String namespace) {
    this.namespace = namespace;
  }

  /**
   * Get the singleton instance of LoggingUtil.
   *
   * @return The LoggingUtil singleton instance
   */
  public static LoggingUtil getInstance() {
    if (instance == null) {
      instance = new LoggingUtil();
    }
    return instance;
  }

  /**
   * Get a namespaced instance of LoggingUtil.
   *
   * @param namespace The namespace for this logger
   * @return A LoggingUtil instance with the specified namespace
   */
  public static LoggingUtil getInstance(String namespace) {
    return new LoggingUtil(namespace);
  }

  /**
   * Enable or disable debug mode.
   *
   * <p>When debug mode is enabled, all debug-level logs are recorded. When disabled, only INFO,
   * WARN, and ERROR levels are logged.
   *
   * @param enabled True to enable debug mode, false to disable
   */
  public static void setDebugMode(boolean enabled) {
    debugMode = enabled;
  }

  /**
   * Check if debug mode is enabled.
   *
   * @return True if debug mode is enabled, false otherwise
   */
  public static boolean isDebugMode() {
    return debugMode;
  }

  /**
   * Enable or disable AdvantageKit logging.
   *
   * @param enabled True to enable, false to disable
   */
  public static void setAdvantageKitEnabled(boolean enabled) {
    advantageKitEnabled = enabled;
  }

  /**
   * Enable or disable SmartDashboard logging.
   *
   * @param enabled True to enable, false to disable
   */
  public static void setSmartDashboardEnabled(boolean enabled) {
    smartDashboardEnabled = enabled;
  }

  /**
   * Get the fully qualified key with namespace.
   *
   * @param key The base key
   * @return The key with namespace prepended
   */
  private String getKey(String key) {
    if (namespace.isEmpty()) {
      return key;
    }
    return namespace + "/" + key;
  }

  // ==================== Number Logging ====================

  /**
   * Log a number value.
   *
   * @param key The key for this value
   * @param value The numerical value to log
   */
  public static void logNumber(String key, double value) {
    getInstance().logNumberInstance(key, value, LogLevel.INFO);
  }

  /**
   * Log a number value with a namespace instance.
   *
   * @param key The key for this value
   * @param value The numerical value to log
   */
  public void logNumber(String key, Number value) {
    logNumberInstance(key, value.doubleValue(), LogLevel.INFO);
  }

  private void logNumberInstance(String key, double value, LogLevel level) {
    if (shouldLog(level)) {
      String fullKey = getKey(key);
      if (advantageKitEnabled) {
        Logger.recordOutput(fullKey, value);
      }
      if (smartDashboardEnabled) {
        SmartDashboard.putNumber(fullKey, value);
      }
    }
  }

  // ==================== Boolean Logging ====================

  /**
   * Log a boolean value.
   *
   * @param key The key for this value
   * @param value The boolean value to log
   */
  public static void logBoolean(String key, boolean value) {
    getInstance().logBooleanInstance(key, value, LogLevel.INFO);
  }

  /**
   * Log a boolean value with a namespace instance.
   *
   * @param key The key for this value
   * @param value The boolean value to log
   */
  public void logBoolean(String key, boolean value) {
    logBooleanInstance(key, value, LogLevel.INFO);
  }

  private void logBooleanInstance(String key, boolean value, LogLevel level) {
    if (shouldLog(level)) {
      String fullKey = getKey(key);
      if (advantageKitEnabled) {
        Logger.recordOutput(fullKey, value);
      }
      if (smartDashboardEnabled) {
        SmartDashboard.putBoolean(fullKey, value);
      }
    }
  }

  // ==================== String Logging ====================

  /**
   * Log a string value.
   *
   * @param key The key for this value
   * @param value The string value to log
   */
  public static void logString(String key, String value) {
    getInstance().logStringInstance(key, value, LogLevel.INFO);
  }

  /**
   * Log a string value with a namespace instance.
   *
   * @param key The key for this value
   * @param value The string value to log
   */
  public void logString(String key, String value) {
    logStringInstance(key, value, LogLevel.INFO);
  }

  private void logStringInstance(String key, String value, LogLevel level) {
    if (shouldLog(level)) {
      String fullKey = getKey(key);
      if (advantageKitEnabled) {
        Logger.recordOutput(fullKey, value);
      }
      if (smartDashboardEnabled) {
        SmartDashboard.putString(fullKey, value);
      }
    }
  }

  // ==================== Array Logging ====================

  /**
   * Log a number array.
   *
   * @param key The key for this value
   * @param values The array to log
   */
  public static void logNumberArray(String key, double[] values) {
    getInstance().logNumberArrayInstance(key, values, LogLevel.INFO);
  }

  /**
   * Log a number array with a namespace instance.
   *
   * @param key The key for this value
   * @param values The array to log
   */
  public void logNumberArray(String key, double[] values) {
    logNumberArrayInstance(key, values, LogLevel.INFO);
  }

  private void logNumberArrayInstance(String key, double[] values, LogLevel level) {
    if (shouldLog(level)) {
      String fullKey = getKey(key);
      if (advantageKitEnabled) {
        Logger.recordOutput(fullKey, values);
      }
      if (smartDashboardEnabled) {
        SmartDashboard.putNumberArray(fullKey, values);
      }
    }
  }

  // ==================== Debug Logging ====================

  /**
   * Log a debug-level number value. Only logged if debug mode is enabled.
   *
   * @param key The key for this value
   * @param value The numerical value to log
   */
  public static void logDebug(String key, double value) {
    getInstance().logNumberInstance(key, value, LogLevel.DEBUG);
  }

  /**
   * Log a debug-level number value with a namespace instance. Only logged if debug mode is
   * enabled.
   *
   * @param key The key for this value
   * @param value The numerical value to log
   */
  public void logDebugNumber(String key, Number value) {
    logNumberInstance(key, value.doubleValue(), LogLevel.DEBUG);
  }

  /**
   * Log a debug-level boolean value. Only logged if debug mode is enabled.
   *
   * @param key The key for this value
   * @param value The boolean value to log
   */
  public static void logDebug(String key, boolean value) {
    getInstance().logBooleanInstance(key, value, LogLevel.DEBUG);
  }

  /**
   * Log a debug-level boolean value with a namespace instance. Only logged if debug mode is
   * enabled.
   *
   * @param key The key for this value
   * @param value The boolean value to log
   */
  public void logDebugBoolean(String key, boolean value) {
    logBooleanInstance(key, value, LogLevel.DEBUG);
  }

  /**
   * Log a debug-level string value. Only logged if debug mode is enabled.
   *
   * @param key The key for this value
   * @param value The string value to log
   */
  public static void logDebug(String key, String value) {
    getInstance().logStringInstance(key, value, LogLevel.DEBUG);
  }

  /**
   * Log a debug-level string value with a namespace instance. Only logged if debug mode is
   * enabled.
   *
   * @param key The key for this value
   * @param value The string value to log
   */
  public void logDebugString(String key, String value) {
    logStringInstance(key, value, LogLevel.DEBUG);
  }

  // ==================== Warning/Error Logging ====================

  /**
   * Log a warning message.
   *
   * @param key The key for this value
   * @param value The string value to log
   */
  public static void logWarn(String key, String value) {
    getInstance().logStringInstance(key, value, LogLevel.WARN);
  }

  /**
   * Log an error message.
   *
   * @param key The key for this value
   * @param value The string value to log
   */
  public static void logError(String key, String value) {
    getInstance().logStringInstance(key, value, LogLevel.ERROR);
  }

  /**
   * Log an exception.
   *
   * @param message The error message
   * @param exception The exception to log
   */
  public static void logException(String message, Throwable exception) {
    String errorMsg = message + ": " + exception.getMessage();
    logError("Errors/" + message, errorMsg);
    exception.printStackTrace();
  }

  // ==================== Helper Methods ====================

  /**
   * Determine if a message at the given level should be logged.
   *
   * @param level The log level to check
   * @return True if the message should be logged, false otherwise
   */
  private boolean shouldLog(LogLevel level) {
    if (level == LogLevel.DEBUG) {
      return debugMode;
    }
    return true; // Always log INFO, WARN, ERROR
  }

  // ==================== Enum ====================

  /** Log level enumeration. */
  private enum LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR
  }
}
