// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private boolean shooting = false;
  SparkFlex shooterLead = new SparkFlex(ShooterConstants.kShooterLeadCanId, MotorType.kBrushless);
  SparkFlex shooterFollow = new SparkFlex(ShooterConstants.kShooterFollowCanId, MotorType.kBrushless);
    RelativeEncoder shooter = shooterLead.getEncoder();

  SparkFlexConfig shooterFollowConfig = new SparkFlexConfig();
  
  
  private final Spark shooterLights = new Spark(0);
  private long lastClickTime = 0;
  private static final long DEBOUNCE_INTERVAL = 200;
  private double current_speed = .2;
  private boolean autoShooting = false;
  private int desiredRPM = 0;
  public void init(){
shooterFollowConfig.follow(40,true);
shooterFollow.configure(shooterFollowConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  

  public void shootConstant(){
    int current_RPMs = (int) shooter.getVelocity();
    shooterLead.set((desiredRPM/6250.0)*(desiredRPM/((double)current_RPMs + 1)));
  }
  public void shoot(int RPMs, int mode, String color) {
    if(color == "green"){
      shooterLights.set(.75);
    }else if(color == "red"){
      shooterLights.set(.61);
    }else if(color == "yellow"){
      shooterLights.set(.69);
    }else if(color == "blue"){
      shooterLights.set(.87);
    }else if(color == "off"){
      shooterLights.set(.99);
    }
    SmartDashboard.putNumber("Light value",shooterLights.get());
    long currentTime = System.currentTimeMillis();

    if(mode == 0){
      if (currentTime - lastClickTime > DEBOUNCE_INTERVAL) {
        lastClickTime = currentTime;
        if(shooting && RPMs == current_speed){
          current_speed = 1000;
          desiredRPM = 1000;
          shooting = false;
          shooterLights.set(.93);
          
        }else{
          current_speed = RPMs;
          desiredRPM = RPMs;
          shooting = true;
        }
        
      }  
    
    }else if(mode == 1 && autoShooting != true){
      current_speed = RPMs;
      desiredRPM = RPMs;
      autoShooting = true;
      shooterLead.set(RPMs/6250.0);
    }else if(mode == 2 && autoShooting != false){
      current_speed = 0;
      desiredRPM = 0;
      autoShooting = false;
      shooterLead.set(0);
    }
    
}


  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Speed", shooter.getVelocity());
    // This method will be called once per scheduler run
  }
}
