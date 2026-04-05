// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {
  private boolean intaking = false;
  private boolean outtaking = false;
  SparkFlex intake = new SparkFlex(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
  private long lastClickTimeIntake = 0;
  private static final long DEBOUNCE_INTERVAL = 200;
  private boolean autoIntake = false;
  
  //m_encoder = arm.getEncoder();
  
  public void intake(double speed, boolean outtake, int mode) {
    if(mode == 0){
      long currentTime = System.currentTimeMillis();
        if (currentTime - lastClickTimeIntake > DEBOUNCE_INTERVAL) {
          
          lastClickTimeIntake = currentTime;
          if(outtake && !outtaking){
            intaking = false;
            outtaking = true;
            intake.set(-speed);
          }else if(!outtake && !intaking){
            outtaking = false;
            intaking = true;
            intake.set(speed);
          }else{
            intaking = false;
            outtaking = false;
            intake.set(0);
          }
        } 
    }else if(mode == 1 && !autoIntake){
        intake.set(speed);
        autoIntake = true;
        intaking = true;
    }else if(mode == 2 && autoIntake){
        autoIntake = false;
        intake.set(0);
        intaking = false;
    }
    SmartDashboard.putBoolean("Intake On", intaking);
  }

  
  

  
  


  /** Creates a new Feeder. */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
