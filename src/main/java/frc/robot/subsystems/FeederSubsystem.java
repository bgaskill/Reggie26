// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;




public class FeederSubsystem extends SubsystemBase {
  SparkFlex launcher = new SparkFlex(FeederConstants.kLauncherCanId, MotorType.kBrushless);
  SparkFlex conveyor = new SparkFlex(FeederConstants.kConveyorCanId, MotorType.kBrushless);
  public void feed(double speed) {
    launcher.set(speed);
    conveyor.set(speed);
    Timer.delay(.5);
    launcher.set(0);
    conveyor.set(0);
    Timer.delay(.2);

  }


  
  


  /** Creates a new Feeder. */
  public FeederSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
