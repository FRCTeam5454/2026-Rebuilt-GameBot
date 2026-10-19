package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.ObsidianCANSparkMax;

public class ClimbSubsystem extends SubsystemBase {
  private ObsidianCANSparkMax m_climbMotor;
  private DigitalInput m_climbUpProx;
  private DigitalInput m_climbDownSwitch;
  private boolean m_homed=false;
 
  //private SparkAbsoluteEncoder m_encoder;

  public ClimbSubsystem(int CanId1) {
    m_climbMotor = new ObsidianCANSparkMax(CanId1, MotorType.kBrushless, true,100);
    m_climbUpProx = new DigitalInput(ClimbConstants.climbUpSwitchDIO);
    m_climbDownSwitch = new DigitalInput(ClimbConstants.climbDownSwitchDIO);
  }

  public boolean isClimbUpLimit() {
    //sensor is closed (true) until it hits the proximity and is open returning false
    return !m_climbUpProx.get();
  }

  public boolean isClimbDownLimit() {
    //sensor is closed (true) until it hits the proximity and is open returning false
    return !m_climbDownSwitch.get();
  }

  /**
   * Drives the climb down until the down limit switch trips, giving up after maxHomeTime seconds.
   * Runs through the scheduler instead of a while loop, so the rest of the robot keeps running.
   */
  public Command homeCommand(double maxHomeTime){
    return Commands.startEnd(()->m_climbMotor.set(ClimbConstants.climbBackSpeed), ()->m_climbMotor.stopMotor(), this)
        .until(this::isClimbDownLimit)
        .withTimeout(maxHomeTime)
        .finallyDo((interrupted)->{
          m_climbMotor.stopMotor();
          if(!interrupted){
            Logger.recordOutput("Climb/HomeTimedOut", !isClimbDownLimit());
            m_homed=true;
          }
        })
        .withName("ClimbHome");
  }

  public void climbGo(double speed) {
    m_climbMotor.set(speed);
  }

  public void stopClimb(){
    m_climbMotor.stopMotor();
  }

  public void setClimbSpeed(double speed){
     m_climbMotor.set(speed);
   
  }

  // Timeouts stop the climb if a limit switch fails, instead of running forever
  public Command climbUpCommand() {
    return Commands.startEnd(()->setClimbSpeed(ClimbConstants.climbForwardSpeed), ()->stopClimb(), this)
        .until(this::isClimbUpLimit)
        .withTimeout(ClimbConstants.climbTimeout)
        .withName("ClimbUp");
  }

  public Command climbDownCommand() {
    return Commands.startEnd(()->setClimbSpeed(ClimbConstants.climbBackSpeed), ()->stopClimb(), this)
        .until(this::isClimbDownLimit)
        .withTimeout(ClimbConstants.climbTimeout)
        .withName("ClimbDown");
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climb At High Limit",m_climbUpProx.get());
    SmartDashboard.putBoolean("Climb At Low Limit",m_climbDownSwitch.get());
    Logger.recordOutput("Climb/UpLimit", isClimbUpLimit());
    Logger.recordOutput("Climb/DownLimit", isClimbDownLimit());
    Logger.recordOutput("Climb/Homed", m_homed);
    Logger.recordOutput("Climb/Output", m_climbMotor.get());
    Logger.recordOutput("Climb/Current", m_climbMotor.getOutputCurrent());
    Logger.recordOutput("Climb/Position", m_climbMotor.getEncoder().getPosition());
  
  }
}