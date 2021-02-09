/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drive;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoyStickDrive extends CommandBase {
  /**
   * Creates the JoyStick Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public JoyStickDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  double leftStick;
  double rightStick;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Mapping for xbox controller
    leftStick = Math.pow(-1* Robot.oi.xbox0.getRawAxis(Robot.oi.XBOX_L_YAXIS),3);
	  rightStick = Math.pow(Robot.oi.xbox0.getRawAxis(Robot.oi.XBOX_R_YAXIS),3);

    // Deadband
		if(Math.abs(leftStick) < .12) {
		 	leftStick = 0;
		}
		if(Math.abs(rightStick) < .12) {
		 	rightStick = 0;
    }
  
    // Set Motor Speed
    Robot.drive.setLeftMotors(leftStick);
    Robot.drive.setRightMotors(rightStick);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
