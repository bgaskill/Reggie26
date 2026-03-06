// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ArmSubsystem extends SubsystemBase {
  boolean configured = false;
  private boolean manual = false;
  private final SparkMax arm = new SparkMax(IntakeConstants.kArmCanId, MotorType.kBrushless);
  private SparkClosedLoopController m_armController = arm.getClosedLoopController();
  private SparkMaxConfig config = new SparkMaxConfig();
  RelativeEncoder armPosition = arm.getAlternateEncoder();
  long lastClickTime = 0;
  public void configureShtuff(){
    config.closedLoop.p(.037).i(0).d(.37);
    config.closedLoop.outputRange(-1, 1);
    arm.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configured = true;
  }
  /** Creates a new Arm. */
  
  public ArmSubsystem() {}
  public void manual(){
    long currentTime = System.currentTimeMillis();
    if(currentTime - lastClickTime > 200){
      manual = !manual;
      lastClickTime = currentTime;
      SmartDashboard.putBoolean("Manual mode", manual);
    }
  }
  public void move(double direction){
   //f(!configured){
     //configureShtuff();
    //}
    if(!manual){
      if(direction<0){
        m_armController.setSetpoint(-80, SparkMax.ControlType.kPosition);
        
      }else if(direction>0){
        m_armController.setSetpoint(0, SparkMax.ControlType.kPosition);
      }
    armPosition = arm.getEncoder(); 
   }else{
    arm.set(direction);
   }
    
    SmartDashboard.putNumber("arm encoder", armPosition.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
