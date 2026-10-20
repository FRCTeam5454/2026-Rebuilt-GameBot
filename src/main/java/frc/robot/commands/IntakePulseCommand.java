package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePulseCommand extends Command {
  private final IntakeSubsystem m_intake;
  private int m_flipCount = 0;
  private int m_HopperPulls = 0;
  private double m_flipSpeed = 0;
  private int m_flipCountLimit = 0;
  private final int kHopperPullLimit = 18;

  public IntakePulseCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_flipCount = 0;
    m_HopperPulls = 0;
    m_flipSpeed = -0.8;
    m_flipCountLimit = 3;
    
    SmartDashboard.putString("IntakePulse/State", "Starting");
    SmartDashboard.putNumber("IntakePulse/HopperPulls", m_HopperPulls);
    SmartDashboard.putNumber("IntakePulse/FlipSpeed", m_flipSpeed);
  }

  @Override
  public void execute() {
    if (m_flipCount > m_flipCountLimit) {
      m_flipCount = 0;
      m_HopperPulls = m_HopperPulls + 1;
    }

    switch (m_HopperPulls) {
      case 0:
        m_flipSpeed = -0.8; // coming in speed
        m_flipCountLimit = 3;
        break;
      case 1:
        m_flipSpeed = 0.8;  // Out speed
        m_flipCountLimit = 2;
        break;
      case 2:
        m_flipSpeed = -0.8;
        m_flipCountLimit = 8;
        break;
      case 3:
        m_flipSpeed = 0.8;
        m_flipCountLimit = 6;
        break;
      case 4:
        m_flipSpeed = -0.8;
        m_flipCountLimit = 12;
        break;
      default:
        if (m_HopperPulls % 2 != 0) {
          m_flipSpeed = 0.8; // Out speed
          m_flipCountLimit = 4;
        } else {
          m_flipSpeed = -0.8; // coming in speed
          m_flipCountLimit = 6;
        }
        break;
    }

    m_flipCount = m_flipCount + 1;

    m_intake.inFold(m_flipSpeed);

    if (m_intake.isinNoFlyZone()) {
      m_intake.stopIntake();
    } else {
      m_intake.runIntake(Constants.IntakeConstants.highSpeed);
    }

    SmartDashboard.putString("IntakePulse/State", "Running");
    SmartDashboard.putNumber("IntakePulse/HopperPulls", m_HopperPulls);
    SmartDashboard.putNumber("IntakePulse/FlipSpeed", m_flipSpeed);
    SmartDashboard.putNumber("IntakePulse/FlipCount", m_flipCount);
    SmartDashboard.putNumber("IntakePulse/FlipCountLimit", m_flipCountLimit);
    SmartDashboard.putBoolean("IntakePulse/RollersActive", !m_intake.isinNoFlyZone());
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopFold();
    m_intake.stopIntake();
    m_intake.SetIntakeOutMode();
    
    SmartDashboard.putString("IntakePulse/State", interrupted ? "Interrupted" : "Finished");
  }

  @Override
  public boolean isFinished() {
    return m_intake.isAtInLimit();
  }
}
