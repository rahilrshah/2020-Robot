// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.Turret;
// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
// import frc.robot.subsystems.Turret;
// public class AutoAim extends CommandBase {
//   // updated var is used to only call limelight target once
//   boolean updated = false;
//   double x;
//   /**
//    * Creates a new AutoAim.
//    */
//   public AutoAim() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(Robot.turret);

//     // FOR TESTING PURPOSES: Turret Setpoint acts like a Limelight
//     //SmartDashboard.putNumber("TurretSetpoint", 10);
//     Turret.Turret.setSelectedSensorPosition(0);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     updated = false;
//     Robot.ledMode.setNumber(0);
//     Robot.tx.setNumber(0);
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   double position;
//   @Override
//   public void execute() 
//   {
//     //double x = SmartDashboard.getNumber("TurretSetpoint", 10);
//     //x += Robot.turret.GetPosition();
//     // if(updated == false){
//     //   x = -Robot.vision.GetX(); 
//     //   updated = true;
//     // }
//     // SmartDashboard.putNumber("X Limelight", x);
//     // Robot.turret.autorotate(1);

//     // add network tables import, initialization for a network table on the limelight docs
//     // position = Robot.vision.GetX();
//     Robot.turret.autorotate(100*Robot.vision.GetX());
//     Robot.turret.getPosition();

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(final boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // if(Robot.turret.getVelocity() <= 0.005)
//     // {
//     //   Robot.ledMode.setNumber(1);
//     //   return true;
//     // }
//     // else return false;
//     return false;
//   }
// }
