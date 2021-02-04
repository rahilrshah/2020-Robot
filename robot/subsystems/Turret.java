/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


;

public class Turret extends SubsystemBase {
  private static Constants consts = new Constants();
  public static TalonFX Turret = new TalonFX(consts.Turret);

  public final int kPIDLoopIdx = 0;
  public final int kTimeoutMs = 20;


  public void initDefaultCommand() {
  }

  public Turret() {
    Turret.configFactoryDefault();
    Turret.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    Turret.config_kF(kPIDLoopIdx, Constants.turretkP, kTimeoutMs);
    Turret.config_kD(kPIDLoopIdx, Constants.turretkD, kTimeoutMs);
    Turret.config_kI(kPIDLoopIdx, Constants.turretkI, kTimeoutMs);
    Turret.config_kP(kPIDLoopIdx, Constants.turretkP, kTimeoutMs);

    Turret.configAllowableClosedloopError(1, kPIDLoopIdx, kTimeoutMs);

    Turret.configNominalOutputForward(0, kTimeoutMs);
    Turret.configNominalOutputReverse(0, kTimeoutMs);
    Turret.configPeakOutputForward(0.2, kTimeoutMs);
    Turret.configPeakOutputReverse(-0.2, kTimeoutMs);
    

    Turret.setInverted(false);
    // SmartDashboard.putNumber("encoder: ", Turret.getSelectedSensorVelocity(0));
  }

  public void autorotate(double position) {

    // Turret.configNominalOutputForward(0, kTimeoutMs);
    // Turret.configNominalOutputReverse(0, kTimeoutMs);
    // Turret.configPeakOutputForward(0.2, kTimeoutMs);
    // Turret.configPeakOutputReverse(-0.2, kTimeoutMs);

    // Turret.configForwardSoftLimitThreshold(10000, 0);
    // Turret.configReverseSoftLimitThreshold(-10000, 0);
    // Turret.configForwardSoftLimitEnable(true, 0);
    // Turret.configReverseSoftLimitEnable(true, 0);

    // Turret.configAllowableClosedloopError(1, kPIDLoopIdx, kTimeoutMs);


    // Turret.config_kD(0, Constants.turretkD, kTimeoutMs);
    // Turret.config_kI(0, Constants.turretkI, kTimeoutMs);
    // Turret.config_kP(0, Constants.turretkP, kTimeoutMs);
    // Turret.config_IntegralZone(0, Constants.turretIntZone, kTimeoutMs);
    
    //linear regression told us conversion lol ::pogchamp::
    Turret.set(ControlMode.Position, position*2048.0/64.0);

    SmartDashboard.putNumber("position", Turret.getSelectedSensorPosition(1)*64.0/4096.0);

    // SmartDashboard.putNumber("Motor Output: ", Turret.getMotorOutputPercent());
    // SmartDashboard.putNumber("Closed Error Loop: ", Turret.getClosedLoopError());
  }


  public void stop(){
    Turret.set(ControlMode.PercentOutput, 0);
  }

  public void rotate(double speed){
    Turret.set(ControlMode.PercentOutput, speed);
  }

  public double getPosition() {
    SmartDashboard.putNumber("position", Turret.getSelectedSensorPosition(0)*64.0/4096.0);
    return Turret.getSelectedSensorPosition(0)*64.0/2048.0;
  }

  // public double getVelocity() {
  //   SmartDashboard.putNumber("speed", Turret.getSelectedSensorVelocity(0) * 600.0 / 2048.0 );
  //   return Turret.getSelectedSensorVelocity(0) * 600.0 / 2048.0;
  // }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
