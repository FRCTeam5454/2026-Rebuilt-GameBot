package frc.robot.utilities;

import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Publishes AdvantageKit log data to NetworkTables, except when connected to the field (FMS).
 * In matches this saves the CPU and network cost of sending every logged value live; the USB
 * log (WPILOGWriter) still records everything. Values the drivers need during a match should
 * go through SmartDashboard, not Logger.recordOutput.
 */
public class NoFMSNT4Publisher implements LogDataReceiver {
  private final NT4Publisher m_publisher = new NT4Publisher();

  @Override
  public void putTable(LogTable table) {
    if (!DriverStation.isFMSAttached()) {
      m_publisher.putTable(table);
    }
  }
}
