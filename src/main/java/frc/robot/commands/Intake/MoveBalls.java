/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveBalls extends CommandBase {
  /**
   * Creates a new MoveBalls.
   */
  public MoveBalls() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.intake);
    addRequirements(Robot.shooter);
  }

  // Called when the command is initially scheduled.r
  @Override
  public void initialize() {
  }
  double leftStick;
  double LTrigger;
  double RTrigger;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // leftStick = Robot.oi.xbox1.getRawAxis(Robot.oi.XBOX_L_YAXIS);
    LTrigger = Robot.oi.xbox0.getRawAxis(Robot.oi.XBOX_L_Trigger);
    RTrigger = Robot.oi.xbox0.getRawAxis(Robot.oi.XBOX_R_Trigger);

    // Robot.intake.downserializer1(LTrigger-RTrigger);
    // Robot.intake.downserializer2(LTrigger-RTrigger);
    Robot.intake.retrieveBall(LTrigger-RTrigger);
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
