/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2212.robot;

import java.awt.geom.Point2D;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.spikes2212.dashboard.DashBoardController;
import com.spikes2212.genericsubsystems.drivetrains.TankDrivetrain;
import com.spikes2212.genericsubsystems.drivetrains.commands.DriveTank;
import com.spikes2212.utils.InvertedConsumer;

import org.usfirst.frc.team2212.odometry.OdometryHandler;
import org.usfirst.frc.team2212.odometry.OdometryUnit;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static OI m_oi;
	public static double leftCalculate, rightCalculate;
	public static SpeedController right_ctrl = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.CAN.RIGHT_BACK), new WPI_TalonSRX(RobotMap.CAN.RIGHT_FRONT));
	public static SpeedController left_ctrl = new SpeedControllerGroup(new WPI_TalonSRX(RobotMap.CAN.LEFT_BACK), new WPI_TalonSRX(RobotMap.CAN.LEFT_FRONT));
	public static final int TICKS_PER_REVOLUTION = 360, maxSpeed = 1, maxAcc = 2, maxJerk = 60;
	public static final double kp = 0.8, ki = 0.0, kd = 0.0, kv = 0.5, ka = 0.0, time = 0.02, WHEEL_DIAMETER = 6 * 0.0254, ROBOT_WIDTH = 0.583;
	// public static Waypoint start = new Waypoint(0,0,0);
	// public static Waypoint middle = new Waypoint(1,0,0);
	// public static Waypoint end = new Waypoint(3,0,0);
	// public static Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time, maxSpeed, maxAcc, maxJerk);
	// public static Trajectory traj = Pathfinder.generate(new Waypoint[] {start, middle, end}, config);
	// public static TankModifier modifier = new TankModifier(traj);
	// public static Trajectory left, right;
	// public static EncoderFollower folLeft, folRight;
	public static TankDrivetrain drivetrain = new TankDrivetrain(left_ctrl::set, new InvertedConsumer(right_ctrl::set));
	public static Encoder leftEncoder = new Encoder(RobotMap.DIO.LEFT_ENCODER_0, RobotMap.DIO.LEFT_ENCODER_1);
	public static Encoder rightEncoder = new Encoder(RobotMap.DIO.RIGHT_ENCODER_0, RobotMap.DIO.RIGHT_ENCODER_1);
	public static DashBoardController dbc = new DashBoardController();
	public static ADIS16448_IMU IMU = new ADIS16448_IMU();
	public static OdometryHandler handler;
	public static OdometryUnit unit;

	public static double x = 2.9, y = 2.212, yaw = 45;
	public static Point2D point = new Point2D.Double(0, 0);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_oi = new OI();
		// modifier.modify(0.583);
		// left = modifier.getLeftTrajectory();
		// right = modifier.getRightTrajectory();
		// folLeft = new EncoderFollower(left);
		// folRight = new EncoderFollower(right);
		// folLeft.configurePIDVA(kp, ki, kd, kv, ka);
		// folRight.configurePIDVA(kp, ki, kd, kv, ka);


		SmartDashboard.putData("Forward", new DriveTank(drivetrain, 0.3, 0.3));
		SmartDashboard.putData("Backward", new DriveTank(drivetrain, -0.3, -0.3));
		SmartDashboard.putData("rotate", new DriveTank(drivetrain, 0.3, -0.3));
		SmartDashboard.putData("joysticks", new DriveTank(drivetrain, OI::getLeft, OI::getRight));
		SmartDashboard.putData("setter", new SetPosition(SetPosition.Position.DOWN_START_HIGH));
		SmartDashboard.putData("setter2", new SetPosition(SetPosition.Position.DOWN_START_LOW));
		SmartDashboard.putData("setter3", new SetPosition(SetPosition.Position.TEAM_THREE_CARGO_SHIP_LOW));
		
		leftEncoder.setDistancePerPulse(15.3 * Math.PI / 36000);
		rightEncoder.setDistancePerPulse(15.3 * Math.PI / 36000);

		dbc.addNumber("leftEncoder", () -> ((double)leftEncoder.getDistance()));
		dbc.addNumber("rightEncoder", () -> ((double)rightEncoder.getDistance()));
		// dbc.addDouble("left output", () -> (leftCalculate));
		// dbc.addDouble("right output", () -> (rightCalculate));

		unit = new OdometryUnit(leftEncoder::getDistance, rightEncoder::getDistance, ROBOT_WIDTH, () -> yaw-IMU.getAngleY());
		handler = new OdometryHandler(unit);

		dbc.addNumber("Robot x", () -> (x));
		dbc.addNumber("Robot y", () -> (y));
		dbc.addNumber("Robot angle", () -> IMU.getAngleY());
		dbc.addNumber("x: odometry displacement", point::getX);
		dbc.addNumber("y: odometry displacement", point::getY);
	}

	@Override
	public void robotPeriodic() {
		super.robotPeriodic();

		point = handler.getDifference();
		x += point.getX();
		y += point.getY();	
		dbc.update();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		IMU.reset();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();

		
	}

	@Override
	public void autonomousInit() {
		leftEncoder.reset();
		rightEncoder.reset();
		}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		dbc.update();
	}

	@Override
	public void teleopInit() {
			leftEncoder.reset();
			rightEncoder.reset();
		}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		dbc.update();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
