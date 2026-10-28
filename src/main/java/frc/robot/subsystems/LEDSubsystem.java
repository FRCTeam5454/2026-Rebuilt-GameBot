package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.robot.utilities.CANdleLib.LEDStrip;

/**
 * Picks an LED pattern from what the robot is doing, highest priority first.
 * Animations run on the CANdle itself, so we only send a CAN frame when the mode changes.
 */
public class LEDSubsystem extends SubsystemBase {

  public enum LedMode {
    DISABLED_NO_ALLIANCE, // rainbow - waiting for the driver station
    DISABLED,             // slow alliance color fade
    CLIMBING,             // rainbow
    SHOOTING,             // fast green strobe - kicker is feeding fuel
    SPINNING_UP,          // orange larson - shot command running, not feeding yet
    INTAKING,             // yellow color flow
    AUTO,                 // purple color flow
    HUB_WARNING,          // white strobe - our hub turns active soon
    HUB_INACTIVE,         // dim alliance color
    HUB_ACTIVE            // solid alliance color
  }

  private static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
  private static final RGBWColor kBlue = new RGBWColor(0, 0, 255, 0);
  private static final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
  private static final RGBWColor kOrange = new RGBWColor(255, 80, 0, 0);
  private static final RGBWColor kYellow = new RGBWColor(255, 200, 0, 0);
  private static final RGBWColor kPurple = new RGBWColor(128, 0, 255, 0);
  private static final RGBWColor kWhite = new RGBWColor(255, 255, 255, 0);

  private static final int kAnimSlot = 0;

  private final CANdle m_candle;
  private final int m_start;
  private final int m_end;

  private final BooleanSupplier m_isClimbing;
  private final BooleanSupplier m_isShooting;
  private final BooleanSupplier m_isFeeding;
  private final BooleanSupplier m_isIntaking;
  private final BooleanSupplier m_isHubActive;
  private final DoubleSupplier m_hubTimeLeft;

  private LedMode m_lastMode = null;
  private Optional<Alliance> m_lastAlliance = Optional.empty();

  /**
   * @param isClimbing true while a climb command is running
   * @param isShooting true while any shot command owns the shooter
   * @param isFeeding true while the kicker is feeding fuel
   * @param isIntaking true while the intake rollers are pulling in
   * @param isHubActive true while our alliance's hub is scoring
   * @param hubTimeLeft seconds until the current hub shift ends
   */
  public LEDSubsystem(CANdle candle, LEDStrip strip,
                      BooleanSupplier isClimbing, BooleanSupplier isShooting, BooleanSupplier isFeeding,
                      BooleanSupplier isIntaking, BooleanSupplier isHubActive, DoubleSupplier hubTimeLeft) {
    m_candle = candle;
    m_start = strip.start();
    m_end = strip.end() - 1;
    m_isClimbing = isClimbing;
    m_isShooting = isShooting;
    m_isFeeding = isFeeding;
    m_isIntaking = isIntaking;
    m_isHubActive = isHubActive;
    m_hubTimeLeft = hubTimeLeft;
  }

  private LedMode pickMode() {
    if (DriverStation.isDisabled()) {
      return DriverStation.getAlliance().isPresent() ? LedMode.DISABLED : LedMode.DISABLED_NO_ALLIANCE;
    }
    if (m_isClimbing.getAsBoolean()) return LedMode.CLIMBING;
    if (m_isFeeding.getAsBoolean()) return LedMode.SHOOTING;
    if (m_isShooting.getAsBoolean()) return LedMode.SPINNING_UP;
    if (m_isIntaking.getAsBoolean()) return LedMode.INTAKING;
    if (DriverStation.isAutonomous()) return LedMode.AUTO;
    if (m_isHubActive.getAsBoolean()) return LedMode.HUB_ACTIVE;
    if (m_hubTimeLeft.getAsDouble() <= LedConstants.hubWarningTime) return LedMode.HUB_WARNING;
    return LedMode.HUB_INACTIVE;
  }

  private RGBWColor allianceColor() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? kRed : kBlue;
  }

  private static RGBWColor scale(RGBWColor color, double scale) {
    return new RGBWColor((int)(color.Red * scale), (int)(color.Green * scale), (int)(color.Blue * scale), 0);
  }

  private void showAnimation(ControlRequest animation) {
    m_candle.setControl(animation);
  }

  private void showSolid(RGBWColor color) {
    //an animation in the slot draws over solid colors, so clear it first
    m_candle.setControl(new EmptyAnimation(kAnimSlot));
    m_candle.setControl(new SolidColor(m_start, m_end).withColor(color));
  }

  private void apply(LedMode mode) {
    switch (mode) {
      case DISABLED_NO_ALLIANCE:
      case CLIMBING:
        showAnimation(new RainbowAnimation(m_start, m_end).withSlot(kAnimSlot)
            .withFrameRate(LedConstants.rainbowFrameRate));
        break;
      case DISABLED:
        showAnimation(new SingleFadeAnimation(m_start, m_end).withSlot(kAnimSlot)
            .withColor(allianceColor()).withFrameRate(LedConstants.fadeFrameRate));
        break;
      case SHOOTING:
        showAnimation(new StrobeAnimation(m_start, m_end).withSlot(kAnimSlot)
            .withColor(kGreen).withFrameRate(LedConstants.strobeFastFrameRate));
        break;
      case SPINNING_UP:
        showAnimation(new LarsonAnimation(m_start, m_end).withSlot(kAnimSlot)
            .withColor(kOrange).withSize(LedConstants.larsonSizeShooting)
            .withBounceMode(LarsonBounceValue.Front).withFrameRate(LedConstants.larsonFrameRate));
        break;
      case INTAKING:
        showAnimation(new ColorFlowAnimation(m_start, m_end).withSlot(kAnimSlot)
            .withColor(kYellow).withFrameRate(LedConstants.flowFrameRate));
        break;
      case AUTO:
        showAnimation(new ColorFlowAnimation(m_start, m_end).withSlot(kAnimSlot)
            .withColor(kPurple).withFrameRate(LedConstants.flowFrameRate));
        break;
      case HUB_WARNING:
        showAnimation(new StrobeAnimation(m_start, m_end).withSlot(kAnimSlot)
            .withColor(kWhite).withFrameRate(LedConstants.strobeSlowFrameRate));
        break;
      case HUB_INACTIVE:
        showSolid(scale(allianceColor(), LedConstants.inactiveHubDim));
        break;
      case HUB_ACTIVE:
        showSolid(allianceColor());
        break;
    }
  }

  @Override
  public void periodic() {
    LedMode mode = pickMode();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    //only talk to the CANdle when something changed
    if (mode != m_lastMode || !alliance.equals(m_lastAlliance)) {
      apply(mode);
      m_lastMode = mode;
      m_lastAlliance = alliance;
    }
    Logger.recordOutput("LEDs/Mode", mode.name());
  }
}
