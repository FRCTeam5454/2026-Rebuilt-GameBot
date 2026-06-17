package frc.robot.commands;
import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.TurretSubsystemPots;
import frc.robot.subsystems.shooter.HubLookUpTable;
import frc.robot.subsystems.shooter.NewShooterSubsystem;
import frc.robot.subsystems.shooter.TurretUtil;
import frc.robot.subsystems.shooter.HubLookUpTable.ShootingParameters;
import frc.robot.subsystems.shooter.TurretUtil.ShotSolution;
import frc.robot.subsystems.shooter.TurretUtil.TargetType;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.Limelight;
/** An example command that uses an example subsystem. */
public class ShotOnTheMoveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  

  private NewShooterSubsystem m_shooter;
  private HopperSubsystem m_hopper;
  private IntakeSubsystem m_intake;
  private TurretSubsystemPots m_turret;
  private Limelight m_limelight;
  private HubLookUpTable m_HubLookUpTable = new HubLookUpTable();
  private boolean m_emptyHopper=false;
  private double m_lastHoodPos=0;
  private double m_lastDistance=60;//24; // default distance to use so will take mid shot if limelight is not responding
  private double m_timeLimit=0;
  private double overrideDistance=0;
  private boolean overrideDistanceFlag=false;
  //private double m_heldTurretAngle=0; // Angle to hold the turret at during shooting
  private enum shooterStates{
    SPINUP,WAIT,SHOOT,NOFUEL,EMPTYHOPPER,SHOOTMORE,NOFUEL2NDCHECK,END
  } 
  private shooterStates m_state;
  private double stateStartTime;
  private double startShootTime;
  private final double kSpinUpTime=1;
  private final double khoodSpeed=Constants.HoodConstants.hoodSpeed;
  private final double khoodDeadband = Constants.HoodConstants.hoodDeadband;
  private double fuelcheckStartTime;
  private final double kfuelcheckWait=2;
  private int m_flipCount=0;
  private int m_flipCountLimit=0;
  private final int kflipCountMax=6;//35;
  private final int kHopperPullLimit=14;
  private CommandSwerveDrivetrain m_swerve;
  private int m_HopperPulls=0;
  //private boolean NoLimeLightMode=0;
  private double m_flipSpeed=0;
  private double kTurretPosDeadband=0.1;
  private boolean m_finished=false;
  private boolean m_stopWhenNoFuel=true;
  public ShotOnTheMoveCommand(CommandSwerveDrivetrain swerve,NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, 
                          TurretSubsystemPots turret,Limelight limelight, double timeLimit, boolean emptyHopper) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_swerve=swerve;
    m_turret=turret;
    m_limelight=limelight;
    m_timeLimit=timeLimit;
    m_stopWhenNoFuel=true;
    overrideDistanceFlag=false;
    
    m_emptyHopper=emptyHopper;
    m_state=shooterStates.SPINUP;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
  }

  public ShotOnTheMoveCommand(TurretSubsystemPots turret,CommandSwerveDrivetrain swerve,NewShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, 
                            double timeLimit) {
    m_hopper=hopper;
    m_shooter=shooter;
    m_intake=intake;
    m_swerve=swerve;
    m_turret=turret;
    m_timeLimit=timeLimit;
    m_stopWhenNoFuel=false;
    m_state=shooterStates.SPINUP;
    addRequirements(m_hopper);
    addRequirements(m_shooter);
    addRequirements(m_intake);
    //DO NOT REQUIRE TURRET OR DRIVE
  }

  private boolean checkNoFuelorFuelTimeLimit(){
    boolean returnValue=false;
    double currentTime;
        if(m_stopWhenNoFuel && m_hopper.getNoFuel()) {
          //System.out.println("No Fuel Detected...");
          returnValue=true;
        }
        //check time limit if the value is greater than zero
        //acts a failsafe if FuelSensor is not working
        currentTime = Timer.getFPGATimestamp();
        if(m_timeLimit>0 && (currentTime>=startShootTime+m_timeLimit)){
          //System.out.println("Shoot Time Limit Reached... Ending Shoot Command");
          returnValue=true;
        }
        return returnValue;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=shooterStates.SPINUP;
    m_finished=false;
    startShootTime=Timer.getFPGATimestamp();
    stateStartTime=startShootTime;
    SmartDashboard.putString("SOTM/State",m_state.toString());
    Logger.recordOutput("Shooter/SOTM/State",m_state.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  double currentTime;
  boolean returnValue=false;

var pose = m_swerve.getPose2d();
ChassisSpeeds robotRelativeSpeeds = m_swerve.getChassisSpeeds();
ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, pose.getRotation());

Translation2d turretFieldVelocity = TurretUtil.getFieldRelativeTurretVelocity(pose, robotRelativeSpeeds);
double turretVelX = turretFieldVelocity.getX();
double turretVelY = turretFieldVelocity.getY();
ShotSolution targetShot = TurretUtil.computeLeadShotSolution(pose,turretVelX,turretVelY,TurretUtil.TargetType.HUB); 
ShotSolution staticShot = TurretUtil.computeShotSolution(pose, TurretUtil.TargetType.HUB);

double targetspeed=targetShot.shooterSpeedRPS;
double hoodPos=targetShot.trajectoryAngleDegrees;
double turretSourceAngle=targetShot.turretAngleDegrees;

//REFERENCE ONLY - DO NOT USE TARGETING
 double StaticTurretAngleTarget=TurretUtil.get5454TurretAngle(pose,TurretUtil.TargetType.HUB);
  //always adjust the angle
   double angle=TurretUtil.get5454TurretAngleFromAngle(turretSourceAngle);
         
  //System.out.println("Shooting On the Move - Speed "  + targetspeed  + 
  //                   "Target Angle:" + angle + " Static Angle:" + StaticTurretAngleTarget +" - State:" + m_state);
  SmartDashboard.putNumber("Turret Util Target Angle",angle);
  double targetPos=m_turret.getTargetMotorPosition(angle);
  SmartDashboard.putNumber("Turret Util Target Pos",targetPos); 
  publishDebug(targetShot, staticShot, robotRelativeSpeeds, fieldRelativeSpeeds, turretVelX, turretVelY, angle, targetPos);
  m_turret.moveMotor(targetPos); 
  
switch(m_state){
    case SPINUP:
         m_shooter.HoodSetPos(hoodPos);
          
         m_shooter.runNewShooter(targetspeed,
                            0);
          m_state=shooterStates.WAIT;
   
    break;
    case WAIT:
              m_shooter.HoodSetPos(hoodPos);
              m_shooter.runNewShooter(targetspeed, 0);
   
   
        if(m_shooter.atTargetSpeed(targetspeed) && checkTurretPos(targetPos)){
            m_state=shooterStates.SHOOT;
            stateStartTime=Timer.getFPGATimestamp();
        } 
        break;
    case SHOOT:
        if(checkTurretPos(targetPos)==false){
          //stop kicker until the angle is alligned
            m_shooter.runNewShooter(targetspeed,
                      0);
            m_state=shooterStates.WAIT;
        } else {          
           m_shooter.runKicker(Constants.ShooterConstants.KickerSpeed);

          m_shooter.runNewShooter(targetspeed,
                            Constants.ShooterConstants.KickerSpeed);
         m_shooter.HoodSetPos(hoodPos);
       
        
          m_hopper.agitate(Constants.HopperConstants.agitateSpeed);
         m_intake.runIntake(Constants.IntakeConstants.highSpeed);
        //STAY IN SHOOT
          if(checkNoFuelorFuelTimeLimit()){
            m_state=shooterStates.NOFUEL;
          }
        
        } 
       break;
    case NOFUEL:
        fuelcheckStartTime=Timer.getFPGATimestamp();
        if(m_emptyHopper){
          m_flipCount=0; 
          m_flipCountLimit=0;
          m_flipSpeed=0;
          m_HopperPulls=0;
          m_state=shooterStates.EMPTYHOPPER;    
          //m_intake.inFold(Constants.IntakeConstants.foldSpeedAutoMode);      
         } else{
          m_state=shooterStates.END;          
         }
      break; 
    case EMPTYHOPPER:
         m_shooter.HoodSetPos(hoodPos);
    
    /*  //System.out.println("Flip Count"+ m_flipCount);
        m_flipCount=m_flipCount+1;
        if (m_flipCount==m_flipCountLimit){
          //make it twice as fast
          m_intake.inFold(Constants.IntakeConstants.foldSpeedAutoMode * 3 *  m_flipSpeed);
          m_flipCount = 0;
          m_flipSpeed=m_flipSpeed*-1; // FLIP SIGN TO REVERSE
          if(m_flipCountLimit<kflipCountMax){
            m_flipCountLimit=m_flipCountLimit+10;
          }
        }
        /*if(m_intake.isAtInLimit() || m_intake.intakeCurrentLimitCheck(Constants.IntakeConstants.ampInStop)){
          m_state=shooterStates.NOFUEL2NDCHECK;
          m_intake.stopFold();
        }*/
        //if flip count (times through the loop) is greeater than limit than move to next hopper
        if(m_flipCount>m_flipCountLimit){
          m_flipCount=0;
          m_HopperPulls=m_HopperPulls+1;
        }
        
        switch(m_HopperPulls){
          case 0:
            m_flipSpeed=-0.8; // coming in speed
            m_flipCountLimit=3;
          break;
          case 1:
            m_flipSpeed=0.8;  //Out speed
            m_flipCountLimit=2;
          break;
          case 2:
            m_flipSpeed=-0.8;
            m_flipCountLimit=8;
          break;
          case 3:
            m_flipSpeed=0.8; 
            m_flipCountLimit=6;
          break;
          case 4:
            m_flipSpeed=-0.8;
            m_flipCountLimit=12;
          break;
          default:
           if (m_HopperPulls % 2 != 0) {  //EDIT if we change hopper pull limit
              m_flipSpeed=0.8; //Out speed which we should start with first in the default case since we end with an inward pull
              m_flipCountLimit=4;
            } else {
              m_flipSpeed=-0.8; // coming in speed
              m_flipCountLimit=6; // We pull in further then we push out incase we had jamed and never pulled in enough to start
            }
          break;
        }


        m_flipCount=m_flipCount+1;
        
        //System.out.println("Flip Count:" + m_flipCount + " Hopper Pulls: "+ m_HopperPulls + " Speed:"+ m_flipSpeed);
   
        m_intake.inFold(m_flipSpeed);
        
        if(m_intake.isinNoFlyZone()){
          m_intake.stopIntake();
        } else {
          m_intake.runIntake(Constants.IntakeConstants.highSpeed);
        }
        
        if(m_HopperPulls>kHopperPullLimit){
          //System.out.println("Stop Folding");
          m_intake.stopFold();
          m_state=shooterStates.SHOOTMORE;
        }
      break;
    case SHOOTMORE:
                  m_shooter.HoodSetPos(hoodPos);
   
        //STAY IN THE LOOP FOREVER UNTIL USER STOPS
     break;
    case NOFUEL2NDCHECK:
        currentTime=Timer.getFPGATimestamp();
        if(currentTime>fuelcheckStartTime+kfuelcheckWait){
          if(checkNoFuelorFuelTimeLimit() ){
            m_state=shooterStates.END;
          }
        }
       break;
    case END:
        m_shooter.hoodHome();
        returnValue=true;
    break;
  }
    m_finished=returnValue;
    SmartDashboard.putString("SOTM/State",m_state.toString());
    Logger.recordOutput("Shooter/SOTM/State",m_state.toString());

      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //System.out.println("Stopping Shooter");
    m_shooter.hoodMoveToZero();
    m_intake.stopFold();
    m_intake.SetIntakeOutMode();
    m_shooter.stopNewShooter(true);
    m_hopper.stopAgitate();
    m_intake.stopIntake();
    SmartDashboard.putBoolean("SOTM/Interrupted",interrupted);
    Logger.recordOutput("Shooter/SOTM/Interrupted",interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }

  private boolean checkTurretPos(double targetPosition){
    if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
      return true;
    }
    boolean returnValue=false;
    double actual=m_turret.getCurrentPosition();
    if(Math.abs(targetPosition-actual)<kTurretPosDeadband){
      returnValue=true;
    }
    return returnValue;
  }

  private void publishDebug(ShotSolution leadShot, ShotSolution staticShot, ChassisSpeeds robotRelativeSpeeds,
                            ChassisSpeeds fieldRelativeSpeeds, double turretVelX, double turretVelY, double turretAngle5454,
                            double targetMotorPosition){
    double leadAngleDelta = leadShot.turretAngleDegrees - staticShot.turretAngleDegrees;

    SmartDashboard.putNumber("SOTM/DistanceMeters",leadShot.distanceMeters);
    SmartDashboard.putNumber("SOTM/StaticDistanceMeters",staticShot.distanceMeters);
    SmartDashboard.putNumber("SOTM/TimeOfFlightSec",leadShot.timeOfFlightSeconds);
    SmartDashboard.putNumber("SOTM/ShooterSpeedRPS",leadShot.shooterSpeedRPS);
    SmartDashboard.putNumber("SOTM/HoodPosition",leadShot.trajectoryAngleDegrees);
    SmartDashboard.putNumber("SOTM/TurretAngleRawDeg",leadShot.turretAngleDegrees);
    SmartDashboard.putNumber("SOTM/TurretAngle5454Deg",turretAngle5454);
    SmartDashboard.putNumber("SOTM/TurretMotorTarget",targetMotorPosition);
    SmartDashboard.putNumber("SOTM/TurretMotorActual",m_turret.getCurrentPosition());
    SmartDashboard.putBoolean("SOTM/TurretAtTarget",checkTurretPos(targetMotorPosition));
    SmartDashboard.putBoolean("SOTM/ShotValid",leadShot.isValid);
    SmartDashboard.putNumber("SOTM/LeadAngleDeltaDeg",leadAngleDelta);
    SmartDashboard.putNumber("SOTM/LeadDistanceMeters",leadShot.leadDistanceMeters);
    SmartDashboard.putNumber("SOTM/AimBehindXMeters",-turretVelX * leadShot.timeOfFlightSeconds);
    SmartDashboard.putNumber("SOTM/AimBehindYMeters",-turretVelY * leadShot.timeOfFlightSeconds);
    SmartDashboard.putNumber("SOTM/PredictedTurretX",leadShot.predictedTurretX);
    SmartDashboard.putNumber("SOTM/PredictedTurretY",leadShot.predictedTurretY);
    SmartDashboard.putNumber("SOTM/RobotRelVxMps",robotRelativeSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("SOTM/RobotRelVyMps",robotRelativeSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("SOTM/RobotOmegaRadPerSec",robotRelativeSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("SOTM/FieldRelVxMps",fieldRelativeSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("SOTM/FieldRelVyMps",fieldRelativeSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("SOTM/TurretFieldVelXMps",turretVelX);
    SmartDashboard.putNumber("SOTM/TurretFieldVelYMps",turretVelY);

    Logger.recordOutput("Shooter/SOTM/DistanceMeters",leadShot.distanceMeters);
    Logger.recordOutput("Shooter/SOTM/StaticDistanceMeters",staticShot.distanceMeters);
    Logger.recordOutput("Shooter/SOTM/TimeOfFlightSec",leadShot.timeOfFlightSeconds);
    Logger.recordOutput("Shooter/SOTM/ShooterSpeedRPS",leadShot.shooterSpeedRPS);
    Logger.recordOutput("Shooter/SOTM/HoodPosition",leadShot.trajectoryAngleDegrees);
    Logger.recordOutput("Shooter/SOTM/TurretAngleRawDeg",leadShot.turretAngleDegrees);
    Logger.recordOutput("Shooter/SOTM/TurretAngle5454Deg",turretAngle5454);
    Logger.recordOutput("Shooter/SOTM/TurretMotorTarget",targetMotorPosition);
    Logger.recordOutput("Shooter/SOTM/TurretMotorActual",m_turret.getCurrentPosition());
    Logger.recordOutput("Shooter/SOTM/TurretAtTarget",checkTurretPos(targetMotorPosition));
    Logger.recordOutput("Shooter/SOTM/ShotValid",leadShot.isValid);
    Logger.recordOutput("Shooter/SOTM/LeadAngleDeltaDeg",leadAngleDelta);
    Logger.recordOutput("Shooter/SOTM/LeadDistanceMeters",leadShot.leadDistanceMeters);
    Logger.recordOutput("Shooter/SOTM/AimBehindXMeters",-turretVelX * leadShot.timeOfFlightSeconds);
    Logger.recordOutput("Shooter/SOTM/AimBehindYMeters",-turretVelY * leadShot.timeOfFlightSeconds);
    Logger.recordOutput("Shooter/SOTM/PredictedTurretX",leadShot.predictedTurretX);
    Logger.recordOutput("Shooter/SOTM/PredictedTurretY",leadShot.predictedTurretY);
    Logger.recordOutput("Shooter/SOTM/RobotRelVxMps",robotRelativeSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Shooter/SOTM/RobotRelVyMps",robotRelativeSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Shooter/SOTM/RobotOmegaRadPerSec",robotRelativeSpeeds.omegaRadiansPerSecond);
    Logger.recordOutput("Shooter/SOTM/FieldRelVxMps",fieldRelativeSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Shooter/SOTM/FieldRelVyMps",fieldRelativeSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Shooter/SOTM/TurretFieldVelXMps",turretVelX);
    Logger.recordOutput("Shooter/SOTM/TurretFieldVelYMps",turretVelY);
  }
  
}

