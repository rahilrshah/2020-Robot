/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final double drivekD = 0;
	public static final double drivekI = 0;
	public static final double drivekP = 0;
	
	// CAN IDs
    public int 	shooter  = 1;
    public int 	FrontRight = 0;
    public int FrontLeft = 3;
    public int 	BackRight = 5;
    public int BackLeft = 2;
    public int Turret = 7;
    public int intake = 6;
    public int serializer2 = 8;
    public int serializer1 = 9;
    public int indexer = 10;  
    
    // Photoe DIO IDs
    public int photoe1 = 0;
    public int photoe2 = 1;

    // Shooter PID Configurations
    public double shooterkP = 0.009;
    public double shooterkF = 0.001;
    public double shooterkI = 0.000095;
    public double shooterkD = 0.0045;
    public int shooterIntZone = 540;


    // TODO: tune turret pid values to not oscillate when the turret turns to an object
    // Turret PID Configurations

    public static double turretkP = .0015;
    public static double turretkI = 0.01;
    public static double turretkD = 0.0;
    public static int turretIntZone = 400;
    //Initial Pose
    public double InitialX = 0;
    public double InitialY = 0;

}
