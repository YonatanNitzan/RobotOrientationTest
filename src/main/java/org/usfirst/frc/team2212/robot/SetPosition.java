package org.usfirst.frc.team2212.robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetPosition extends InstantCommand {
    public static final double PI = Math.PI;
    private Position position;
    private double yawAddition = 0;
    
    public enum Position{
        UP_START_HIGH(265, 256, 0), UP_START_LOW(333, 256, 0),
        MIDDLE_START(333, 333, 0), DOWN_START_HIGH(265, 410, 0),
        DOWN_START_LOW(333, 410, 0), NEUTRAL_CARGO_SHIP_HIGH(473, 314, 0),
        NEUTRAL_CARGO_SHIP_LOW(473, 342, 0), TEAM_ONE_CARGO_SHIP_HIGH(550, 270, -PI/2),
        TEAM_ONE_CARGO_SHIP_LOW(550, 386, PI/2), TEAM_TWO_CARGO_SHIP_HIGH(588, 270, -PI/2),
        TEAM_TWO_CARGO_SHIP_LOW(588, 386, PI/2), TEAM_THREE_CARGO_SHIP_HIGH(627, 270, -PI/2),
        TEAM_THREE_CARGO_SHIP_LOW(627, 386, PI/2), HIGH_PICKUP(236, 87, -PI), LOW_PICKUP(236, 569, -PI),
        //UP_ROCKET_RIGHT(), UP_ROCKET_MIDDLE(),
        //UP_ROCKET_LEFT(), DOWN_ROCKET_RIGHT(),
        /*DOWN_ROCKET_MIDDLE(), DOWN_ROCKET_LEFT() */;
        double x, y;
        double yaw;
        Position(double x, double y, double yaw){
            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }
    }
    public SetPosition(Position position){
        this.position = position;
    }
    @Override
    protected void initialize(){
        Robot.leftEncoder.reset();
        Robot.rightEncoder.reset();
        Robot.IMU.reset();
        Robot.x = position.x/100;
        Robot.y = position.y/100;
        robot.yaw = Math.toDegrees(position.yaw);
    }
}
