/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

// import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class Shooter extends SubsystemBase {
  private static Constants consts = new Constants();
  public static TalonFX shooter = new TalonFX(consts.shooter);
  public static VictorSPX indexer = new VictorSPX(consts.indexer);

  public final int kPIDLoopIdx = 0;
  public final int kTimeoutMs = 30;


  public static PowerDistributionPanel pdp = new PowerDistributionPanel();


  public Shooter() {
    shooter.configFactoryDefault();
    shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    shooter.setSensorPhase(true);
    

    shooter.setInverted(true);

    
  }



  public void run(final double speed){
    
  

    shooter.setInverted(true);
    shooter.set(ControlMode.Velocity, speed * 4096 / 1200);// 4096/1200
    SmartDashboard.putNumber("encoder: ", shooter.getSelectedSensorVelocity(1));

    
  }

  public void norun(){
    shooter.set(ControlMode.PercentOutput,0);
    indexer.set(ControlMode.PercentOutput,0);
  }

  public void index(double speed){
    indexer.set(ControlMode.PercentOutput , speed);
  }

  public void indexstop(){
    indexer.set(ControlMode.PercentOutput , 0);
  }

  public void getVelocity(){
    SmartDashboard.putNumber("Shooter Velocity: ", shooter.getSelectedSensorVelocity(1)* -1200 / 4096);//-1200/4096
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
