/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2212.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	public interface CAN{
		public static final int RIGHT_BACK = 0;
		public static final int RIGHT_FRONT = 1;
		public static final int LEFT_BACK = 4;
		public static final int LEFT_FRONT = 5;
	}
	public interface DIO{
		public static final int LEFT_ENCODER_0 = 0;
		public static final int LEFT_ENCODER_1 = 1;
		public static final int RIGHT_ENCODER_0 = 2;
		public static final int RIGHT_ENCODER_1 = 3;
	}
}
