package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{
  private boolean manual = false;
    private final SparkFlex climberLead = new SparkFlex(ClimbConstants.kRightClimbCanId, MotorType.kBrushless);
    private final SparkFlex climberFollow = new SparkFlex(ClimbConstants.kLeftClimbCanId, MotorType.kBrushless);
  RelativeEncoder climbPosition = climberLead.getEncoder();
  SparkFlexConfig climberFollowConfig = new SparkFlexConfig();
  public void init(){
    climberFollowConfig.follow(ClimbConstants.kRightClimbCanId,true);
    climberFollow.configure(climberFollowConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  private SparkClosedLoopController m_climbController = climberLead.getClosedLoopController();
   private SparkMaxConfig config = new SparkMaxConfig();
   boolean configured = false;
  public void configureShtuff(){
    config.closedLoop.p(.037).i(0).d(.37);
    config.closedLoop.outputRange(-1, 1);
    climberLead.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configured = true;
  }
  public void climbUp(){
    if(!configured){
      configureShtuff();
    }
    m_climbController.setSetpoint(-84, SparkMax.ControlType.kPosition);   
  }
  public void climbDown(){
    if(!configured){
      configureShtuff();
    }
    m_climbController.setSetpoint(0, SparkMax.ControlType.kPosition);   
  }
  public void manual(){
    manual = true;
  }
  public void climb(double direction){
    if(manual == true){
      climberLead.set(direction);
    }
    
    SmartDashboard.putNumber("Climber Position",climbPosition.getPosition());
  }
}
