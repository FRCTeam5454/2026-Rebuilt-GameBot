package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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

  public Command homeClimbCommand(double maxHomeTime){
    return Commands.startEnd(
        () -> m_climbMotor.set(ClimbConstants.climbBackSpeed),
        () -> {
            m_climbMotor.stopMotor();
            m_homed = true;
        },
        this
    ).until(this::isClimbDownLimit).withTimeout(maxHomeTime);
  }

  // Deprecated/Blocking version replaced by command
  public void homeClimb(double maxHomeTime) {
      // Intentionally left blank to not break RobotContainer before it's updated
  }

  public void climbGo(double speed) {
    m_climbMotor.set(speed);
  }

  public void stopClimb(){
    m_climbMotor.stopMotor();
  }

  public void extendClimb() {
    
    m_climbMotor.set(ClimbConstants.climbForwardSpeed);
    while (!isClimbUpLimit()){
      //just chill out and wait for robbot
    }
    if (isClimbUpLimit()) {
      m_climbMotor.stopMotor();
    }
  }
  public void setClimbSpeed(double speed){
     m_climbMotor.set(speed);
   
  }
  
  public void retractClimb() {
    m_climbMotor.set(ClimbConstants.climbBackSpeed);
    while (!isClimbDownLimit()){
      //just chill out and wait for robbot
    }
    if (isClimbDownLimit()) {
      m_climbMotor.stopMotor();
    }
  }

  public Command climbUpCommand() {
    return Commands.startEnd(
        () -> m_climbMotor.set(ClimbConstants.climbForwardSpeed),
        () -> m_climbMotor.stopMotor(),
        this
    ).until(this::isClimbUpLimit);
  }

  public Command climbDownCommand() {
    return Commands.startEnd(
        () -> m_climbMotor.set(ClimbConstants.climbBackSpeed),
        () -> m_climbMotor.stopMotor(),
        this
    ).until(this::isClimbDownLimit);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climb At High Limit",m_climbUpProx.get());
    SmartDashboard.putBoolean("Climb At Low Limit",m_climbDownSwitch.get());
  
  }
}