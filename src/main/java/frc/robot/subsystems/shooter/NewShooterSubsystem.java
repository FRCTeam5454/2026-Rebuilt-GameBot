// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.ObsidianCANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;

public class NewShooterSubsystem extends SubsystemBase {
    private TalonFX m_1shooterMotor;
    private TalonFX m_2shooterMotor;
    private TalonFX m_hoodMotor;
    private CANcoder m_hoodCoder;
    private double testLastHood=0;
    private Pose2d m_pose;
    private GenericEntry m_kickerRunningEntry;
    private GenericEntry m_speedAdjusterEntry;
   private PositionVoltage m_request= new PositionVoltage(0).withSlot(0);
 //  private MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    //private TalonFX m_kickerMotor;
    private ObsidianCANSparkMax m_kickerMotor;
    private double m_simTargetVelocity = 0;
    private double m_simTargetHood = 0.05;
  public NewShooterSubsystem(int shooter1CANID, int shooter2CANID, int kickerCANID,int hoodCANID) {
    m_hoodCoder = new CANcoder(Constants.HoodConstants.hoodCoderCANID);
    m_1shooterMotor = new TalonFX(shooter1CANID);
    configureShootermotor(m_1shooterMotor);
    m_1shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    m_2shooterMotor = new TalonFX(shooter2CANID);
    configureShootermotor(m_2shooterMotor);
    m_2shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    m_2shooterMotor.setControl(new Follower(m_1shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    m_hoodMotor = new TalonFX(hoodCANID);
    
    m_kickerMotor = new ObsidianCANSparkMax(kickerCANID, MotorType.kBrushless,true,Constants.k70Amp);
    
        /* Configure CANcoder to zero the magnet appropriately */
    CANcoderConfiguration hoodCoder_cfg = new CANcoderConfiguration();
    hoodCoder_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.8;//1.0; // unsigned [0,1) range, so 1.0 is the same as 0.0, which means the discontinuity is at the wrap-around point
    hoodCoder_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    hoodCoder_cfg.MagnetSensor.withMagnetOffset(Rotations.of(Constants.HoodConstants.hoodOffset));
    m_hoodCoder.getConfigurator().apply(hoodCoder_cfg);
    
  
 
   configureMotionMagicHood(); //on Hood Motor //
   
    m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);
   
    m_kickerRunningEntry = Shuffleboard.getTab("Shooter")
                              .add("Kicker Running", false)
                              .getEntry();
    m_speedAdjusterEntry = Shuffleboard.getTab("Shooter")
                              .add("Speed Adjuster", 1.0)
                              .getEntry();
}
private void configureMotionMagicHood(){
  // in init function
TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
talonFXConfigs.Feedback.FeedbackRemoteSensorID = Constants.HoodConstants.hoodCoderCANID;
talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; // Set the feedback source to the CANcoder
System.out.println("Feedback config setting "+ talonFXConfigs.Feedback.FeedbackRemoteSensorID);  

// set slot 0 gains
Slot0Configs slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kP = 16; // An error of 1 rps results in 0.11 V output
slot0Configs.kI = 0.0; // An error of 1 rps increases output by 0.5 V each second
slot0Configs.kD = 0.05; // An acceleration of 1 rps/s results in 0.01 V output

m_hoodMotor.getConfigurator().apply(talonFXConfigs);
}


  public double getHoodPos(){
    double currentPos=m_hoodCoder.getAbsolutePosition().getValueAsDouble();
    if(currentPos<0.95){
      return currentPos;
    }else {
      //went slightly below zero
      return 0.0;
    }
  }

  public double getShooterSpeed(){
    return m_1shooterMotor.getVelocity().getValueAsDouble();
  }
  public void setPose(Pose2d pose){
      m_pose=pose;
   //   //System.out.println("Pose Set" + m_pose.getX());
      
    }
    public Pose2d getRobotPose(){
   //   //System.out.println("Reuurning Pose : " + m_pose.getX()); 
      return m_pose;
    }
  
  
  public boolean checkHoodPos(double hoodTarget, double hoodSpeed,double deadband){
    double hoodDiff=Math.abs(getHoodPos()-hoodTarget);
    return (hoodDiff<=deadband);
  }

  

  public void HoodSetPos(double hoodTarget) {
    m_simTargetHood = hoodTarget;
    System.out.println("Hood Target is " + hoodTarget);
    m_hoodMotor.setControl(m_request.withPosition(hoodTarget));
  }

  public void primeMotors(double primeSpeed){
    //System.out.println("Shooter Priming:" + primeSpeed);
    runShooterVelocity(primeSpeed);
  }
  private void hoodMoveToPosition(double hoodTarget){
     HoodSetPos(hoodTarget);
  }
  
  public void hoodMoveToZero(){
    double hoodTarget=0.05;  // go close to zero
    hoodMoveToPosition(hoodTarget);
  }
 

  public void runNewShooter(double speed,double kickerSpeed) {
    //System.out.println("Shooter Spin:" + speed);
    runShooterVelocity(speed);
    m_kickerMotor.set(kickerSpeed);
  }
public void runKicker(double kickerSpeed){
  m_kickerMotor.set(kickerSpeed);
}
public boolean atTargetSpeed(double targetSpeed){
  if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
    return true;
  }
  double currentSpeed=m_1shooterMotor.getVelocity().getValueAsDouble();
  double speedDiff = Math.abs(currentSpeed-targetSpeed);
  //System.out.println("Compare Shooter Speed:" + currentSpeed + " - " + targetSpeed);
  if(currentSpeed>targetSpeed){
    //allow greater deadband for going over shot
     return (speedDiff<Constants.ShooterConstants.shooterVelocityHighDeadband);
  } else {
       return (speedDiff<Constants.ShooterConstants.shooterVelocityLowDeadband);
  }
 
}
public void runShooterVelocity(double targetSpeed){
m_simTargetVelocity = targetSpeed;
// Torque-current bang-bang
//m_1shooterMotor.setControl(new MotionMagicVelocityVoltage(targetSpeed));
//m_1shooterMotor.setControl(new MotionMagicVelocityVoltage(-targetSpeed));
//replaced FOC 
m_1shooterMotor.setControl(new VelocityVoltage(targetSpeed));


/*    m_1shooterMotor.setControl(new VelocityTorqueCurrentFOC(0)
                  .withVelocity(targetSpeed)
                  .withFeedForward(0.1));
    m_2shooterMotor.setControl(new VelocityTorqueCurrentFOC(0)
                  .withVelocity(-targetSpeed)
                  .withFeedForward(0.1));
   */
  }
  
  public boolean isShooterAtIdle(){
    //if shoter is below idle threshold, we can consider it at idle, which means it's not spinning enough to shoot, and we can stop it without worrying about cutting power to a spinning flywheel
    double currentSpeed=m_1shooterMotor.getVelocity().getValueAsDouble();
    return (currentSpeed<Constants.ShooterConstants.IdleSpeedThreshold);
  }

  public void stopNewShooter(boolean idleMode){
    //System.out.println("stopping shooter");
    
    if(idleMode){
    
      runShooterVelocity(Constants.ShooterConstants.IdleSpeed);
    }else {
      m_1shooterMotor.stopMotor();
      m_2shooterMotor.stopMotor();
    }
    m_kickerMotor.stopMotor();
    hoodMoveToZero();

  }

  
  private void configureShootermotor(TalonFX motor){
    TalonFXConfigurator configurator = motor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Closed-loop Velocity PID + Feedforward tuning
    // kS: static friction feedforward in Volts
    // kV: velocity feedforward in Volts per RPS (~12V / ~104 RPS)
    // kP: proportional feedback for rapid error correction
    // kD: derivative feedback to prevent velocity overshoot
    config.Slot0.kS = 0.15;
    config.Slot0.kV = 0.115;
    config.Slot0.kP = 0.6;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.01;
    
    config.TorqueCurrent.PeakForwardTorqueCurrent = 800.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -800.0;
    config.MotorOutput.PeakForwardDutyCycle = 1.0;
    config.MotorOutput.PeakReverseDutyCycle = -1.0;
    configurator.apply(config);
    
    // Apply and enable current limits
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimit = 80;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = 60;
    currentLimits.SupplyCurrentLimitEnable = true;
    configurator.apply(currentLimits);
  }
  
  
  public void stopHood(){
    m_hoodMotor.stopMotor();
  }
  public void hoodHome(){
    hoodMoveToZero();
  }
  
  public Command shootCommand(){
    return Commands.startEnd(    ()->runNewShooter(ShooterConstants.shootSpeed,
                                    ShooterConstants.KickerSpeed),
                                           ()->stopNewShooter(true),
                                           this);
  }
  public Command shootonCommand(){
    return Commands.runOnce(    ()->runNewShooter(ShooterConstants.shootSpeed,
                                    ShooterConstants.KickerSpeed),this);
  }
  public Command shootoffCommand(){
    return Commands.runOnce(    ()->stopNewShooter(true),this);
  }
public Command shutdownCommand(){
    return Commands.runOnce(    ()->stopNewShooter(false),this);
  }
    @Override
  public void periodic(){
    if (m_speedAdjusterEntry != null) {
      SmartDashboard.putNumber("Speed Adjuster:", m_speedAdjusterEntry.getDouble(1.0));
    }

    SmartDashboard.putNumber("Shooter Velocity", m_1shooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hood CanCoder Value",m_hoodCoder.getAbsolutePosition().getValueAsDouble());  
    SmartDashboard.putNumber("Hood 'Pos'",getHoodPos());  
    boolean kickerRunning = Math.abs(m_kickerMotor.get()) > 0.05;
    SmartDashboard.putBoolean("Kicker Running", kickerRunning);
    SmartDashboard.putNumber("Kicker Speed", m_kickerMotor.get());
    if (m_kickerRunningEntry != null) {
      m_kickerRunningEntry.setBoolean(kickerRunning);
    }
    Logger.recordOutput("Shooter/KickerRunning", kickerRunning);
    Logger.recordOutput("Shooter/KickerSpeed", m_kickerMotor.get());
    Logger.recordOutput("Shooter/ShooterMotor1Velocity", m_1shooterMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/ShooterMotor2Velocity",  m_2shooterMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Shooter/ShooterMotor1Current", m_1shooterMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/ShooterMotor2Current",  m_2shooterMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/Hood CanCoder Value", m_hoodCoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput("Shooter/Hood 'Pos'",getHoodPos());
   
  }

  @Override
  public void simulationPeriodic() {
    // Set simulated speeds and positions directly to eliminate lag in simulation
    m_1shooterMotor.getSimState().setRotorVelocity(m_simTargetVelocity);
    m_2shooterMotor.getSimState().setRotorVelocity(-m_simTargetVelocity);
    m_hoodCoder.getSimState().setRawPosition(m_simTargetHood);
  }
}