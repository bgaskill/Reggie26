// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private boolean shooting = false;
  SparkFlex shooterLead = new SparkFlex(ShooterConstants.kShooterLeadCanId, MotorType.kBrushless);
  SparkFlex shooterFollow = new SparkFlex(ShooterConstants.kShooterFollowCanId, MotorType.kBrushless);

  SparkFlexConfig shooterFollowConfig = new SparkFlexConfig();
  
  

  private long lastClickTime = 0;
  private static final long DEBOUNCE_INTERVAL = 200;
  private double current_speed = .2;
  private boolean autoShooting = false;

  public void init(){
shooterFollowConfig.follow(40,true);
shooterFollow.configure(shooterFollowConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }


  public void shoot(double speed, int mode) {

    long currentTime = System.currentTimeMillis();

    if(mode == 0){
      if (currentTime - lastClickTime > DEBOUNCE_INTERVAL) {
        lastClickTime = currentTime;
        if(shooting && speed == current_speed){
          current_speed = .2;
          shooterLead.set(.2);
          shooting = false;
          
        }else{
          current_speed = speed;
          shooterLead.set(speed);
          shooting = true;
        }
        
      }  
    
    }else if(mode == 1 && autoShooting != true){
      current_speed = speed;
      shooterLead.set(speed);
      autoShooting = true;
    }else if(mode == 2 && autoShooting != false){
      current_speed = 0;
      shooterLead.set(0);
      autoShooting = false;
    }
    
}
    


  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
