package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{
    private final SparkFlex climber = new SparkFlex(ClimbConstants.kClimbCanId, MotorType.kBrushless);
  AbsoluteEncoder climbPosition = climber.getAbsoluteEncoder();
  public void climb(double direction){
    climber.set(direction);
    climbPosition = climber.getAbsoluteEncoder();
    SmartDashboard.putNumber("Climber Position",climbPosition.getPosition());
    }
}
