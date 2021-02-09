/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.geometry.*;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Drive extends SubsystemBase {
  // Create variables
  private static Constants consts = new Constants();
  public static TalonFX FrontRight = new TalonFX(consts.FrontRight);
  public static TalonFX FrontLeft = new TalonFX(consts.FrontLeft);
  public static TalonFX BackRight = new TalonFX(consts.BackRight);
  public static TalonFX BackLeft = new TalonFX(consts.BackLeft);

  // Create NavX
  // public static AHRS navx = new AHRS(SPI.Port.kMXP);


  // public final int kPIDLoopIdx = 1;
  // public final int kTimeoutMs = 30;

  // public  double kD = consts.drivekD;
  // public  double kI = consts.drivekI;
  // public  double kP = consts.drivekP;
	
  // Create Differential Drive Odometry Object and pose
  DifferentialDriveOdometry m_odometry;
  Pose2d m_pose = new Pose2d(consts.InitialX, consts.InitialY, new Rotation2d());

  public Drive()   {
    // Config Encoders
    // FrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // FrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // BackRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // BackLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    //we must reset encoders before instantiating the odometry
    // resetEncoders();
    
    //create odometry
    // m_odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d(consts.InitialX, consts.InitialY, new Rotation2d()));
  }

  // Set Right Speeds on Drive Train
  // public void setRightSpeed(double speed) {
  //   speed = -speed;
	// 	if(speed<-1) speed =-1;
	// 	if(speed>1) speed=1;
  //   BackRight.set(TalonFXControlMode.PercentOutput,speed);
  //   FrontRight.set(TalonFXControlMode.PercentOutput,speed);
  // }

  // Set Speeds on Drive Train (Tank Drive)
  // public void setSpeed(double speed, double speed1) {
    // Ensures speed and speed1 are within range [-1,1]
		// if(speed<-1) speed =-1;
    // if(speed>1) speed=1;
    // if(speed1<-1) speed1 =-1;
    // if(speed1>1) speed1=1;

    // SmartDashboard.putNumber("Roll: ", navx.getRoll());
    // if(navx.getRoll() <= -10){
    //   BackLeft.set(TalonFXControlMode.PercentOutput,-0.15);
    //   FrontLeft.set(TalonFXControlMode.PercentOutput,-0.15);
    //   BackRight.set(TalonFXControlMode.PercentOutput,0.15);
		//   FrontRight.set(TalonFXControlMode.PercentOutput,0.15);
    // }
    // else if(navx.getRoll() >= 10){
    //   BackLeft.set(TalonFXControlMode.PercentOutput,0.15);
    //   FrontLeft.set(TalonFXControlMode.PercentOutput,0.15);
    //   BackRight.set(TalonFXControlMode.PercentOutput,-0.15);
		//   FrontRight.set(TalonFXControlMode.PercentOutput,-0.15);
    // }
    // else{
    //   BackLeft.set(TalonFXControlMode.PercentOutput,speed);
    //   FrontLeft.set(TalonFXControlMode.PercentOutput,speed);
    //   BackRight.set(TalonFXControlMode.PercentOutput,speed1);
		//   FrontRight.set(TalonFXControlMode.PercentOutput,speed1);
    // }


  // }


  public void setLeftMotors(double speed){
    FrontLeft.set(ControlMode.PercentOutput,speed);
    BackLeft.set(ControlMode.PercentOutput,speed);
  }
  public void setRightMotors(double speed){
    FrontRight.set(ControlMode.PercentOutput,speed);
    BackRight.set(ControlMode.PercentOutput,speed);
  }

  // public void PIDdrive(double distance){
  //   kF = consts.drivekF;
  //   kD = consts.drivekD;
  //   kI = consts.drivekI;
  //   kP = consts.drivekP;
  //   FrontRight.configNominalOutputForward(0, kTimeoutMs);
  //   FrontRight.configNominalOutputReverse(0, kTimeoutMs);
  //   FrontRight.configPeakOutputForward(0.5, kTimeoutMs);
  //   FrontRight.configPeakOutputReverse(-0.5, kTimeoutMs);

  //   FrontRight.configForwardSoftLimitThreshold(10000, 0);
  //   FrontRight.configReverseSoftLimitThreshold(-10000, 0);
  //   FrontRight.configForwardSoftLimitEnable(true, 0);
  //   FrontRight.configReverseSoftLimitEnable(true, 0);

  //   FrontRight.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

  //   FrontRight.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
  //   FrontRight.config_kD(0, kD, kTimeoutMs);
  //   FrontRight.config_kI(0, kI, kTimeoutMs);
  //   FrontRight.config_kP(0, kP, kTimeoutMs);


  //   FrontLeft.configNominalOutputForward(0, kTimeoutMs);
  //   FrontLeft.configNominalOutputReverse(0, kTimeoutMs);
  //   FrontLeft.configPeakOutputForward(0.5, kTimeoutMs);
  //   FrontLeft.configPeakOutputReverse(-0.5, kTimeoutMs);

  //   FrontLeft.configForwardSoftLimitThreshold(10000, 0);
  //   FrontLeft.configReverseSoftLimitThreshold(-10000, 0);
  //   FrontLeft.configForwardSoftLimitEnable(true, 0);
  //   FrontLeft.configReverseSoftLimitEnable(true, 0);

  //   FrontLeft.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

  //   FrontLeft.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
  //   FrontLeft.config_kD(0, kD, kTimeoutMs);
  //   FrontLeft.config_kI(0, kI, kTimeoutMs);
  //   FrontLeft.config_kP(0, kP, kTimeoutMs);




  //   BackLeft.configNominalOutputForward(0, kTimeoutMs);
  //   BackLeft.configNominalOutputReverse(0, kTimeoutMs);
  //   BackLeft.configPeakOutputForward(0.5, kTimeoutMs);
  //   BackLeft.configPeakOutputReverse(-0.5, kTimeoutMs);

  //   BackLeft.configForwardSoftLimitThreshold(10000, 0);
  //   BackLeft.configReverseSoftLimitThreshold(-10000, 0);
  //   BackLeft.configForwardSoftLimitEnable(true, 0);
  //   BackLeft.configReverseSoftLimitEnable(true, 0);

  //   BackLeft.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

  //   BackLeft.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
  //   BackLeft.config_kD(0, kD, kTimeoutMs);
  //   BackLeft.config_kI(0, kI, kTimeoutMs);
  //   BackLeft.config_kP(0, kP, kTimeoutMs);




  //   BackRight.configNominalOutputForward(0, kTimeoutMs);
  //   BackRight.configNominalOutputReverse(0, kTimeoutMs);
  //   BackRight.configPeakOutputForward(0.5, kTimeoutMs);
  //   BackRight.configPeakOutputReverse(-0.5, kTimeoutMs);

  //   BackRight.configForwardSoftLimitThreshold(10000, 0);
  //   BackRight.configReverseSoftLimitThreshold(-10000, 0);
  //   BackRight.configForwardSoftLimitEnable(true, 0);
  //   BackRight.configReverseSoftLimitEnable(true, 0);

  //   BackRight.configAllowableClosedloopError(0, kPIDLoopIdx, kTimeoutMs);

  //   BackRight.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
  //   BackRight.config_kD(0, kD, kTimeoutMs);
  //   BackRight.config_kI(0, kI, kTimeoutMs);
  //   BackRight.config_kP(0, kP, kTimeoutMs);


  //   FrontRight.set(ControlMode.Position, distance);
  //   FrontLeft.set(ControlMode.Position, distance);
  //   BackRight.set(ControlMode.Position, distance);
  //   BackLeft.set(ControlMode.Position, distance);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // // Don't exactly know if it's Yaw or Pitch
    // SmartDashboard.putNumber("YAW: ", navx.getYaw());
    // Rotation2d gyroAngle = Rotation2d.fromDegrees(-navx.getYaw());
    
    // //Average position of the two encoders on each side
    // double right_position = ((FrontRight.getSelectedSensorPosition(0)*64.0/4096.0) + (BackRight.getSelectedSensorPosition(0)*64.0/4096.0))/2;
    // double left_position = ((FrontLeft.getSelectedSensorPosition(0)*64.0/4096.0) + (BackLeft.getSelectedSensorPosition(0)*64.0/4096.0))/2;
    
    // // Update Odometry
    // m_pose = m_odometry.update(gyroAngle, left_position, right_position);
	  
	  // SmartDashboard.putNumber("Drive X", m_pose.getTranslation().getX());
    // SmartDashboard.putNumber("Drive Y", m_pose.getTranslation().getY());
    // SmartDashboard.putNumber("Drive Angle", m_pose.getRotation().getDegrees());
  }

  // Get Encoder Values from Drive Train motors
  // public void getDriveValues(){
  //   SmartDashboard.putNumber("Front Right Speed: ", FrontRight.getSelectedSensorVelocity(0)* 600 / 2048);
  //   SmartDashboard.putNumber("Front Left Speed: ", FrontLeft.getSelectedSensorVelocity(0)* 600 / 2048);
  //   SmartDashboard.putNumber("Back Right Speed: ", BackRight.getSelectedSensorVelocity(0)* 600 / 2048);
  //   SmartDashboard.putNumber("Back Left Speed: ", BackLeft.getSelectedSensorVelocity(0)* 600 / 2048);
  // }

  // public void resetEncoders(){
  //   FrontRight.setSelectedSensorPosition(0);
  //   FrontLeft.setSelectedSensorPosition(0);
  //   BackRight.setSelectedSensorPosition(0);
  //   BackLeft.setSelectedSensorPosition(0);
  // }

  // Retrieve Pose Estimation from odometry
  public Pose2d getPosePosition(){
    return m_pose;
  }

  // Retrieve Position of frontright motor
  // Double check these calcs are correct
  // public double getPosition(){
  //   return FrontRight.getSelectedSensorPosition(0)*64.0/4096.0;
  // }
}
