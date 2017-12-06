/**
 * commands/AutonomousCommand.java
 *
 * Created by Nicholas Hubbard on 11/07/2017.
 *
 * Copyright (c) 2017 Team 1701 (Robocubs)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.usfirst.frc.team1701.robot.commands;
import org.usfirst.frc.team1701.robot.Robot;
import org.usfirst.frc.team1701.robot.RobotMap;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable; // Eventually replaceable by ZeroMQ.
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Autonomous driving command.
 */
public class AutonomousCommand extends Command {
    public static final double DRIVE_FORWARD_DISTANCE = 135.0;
    public static final double AUTO_DRIVE_SPEED = -1.0;
    public static final double AUTO_TURN_CORRECT = -.03;
    public static final double DRIVE_CORRECTION = 1.3;
    private double actualDriveSpeed = 0;
    private final double AUTO_TURN_SPEED = .3;
    private boolean isFinished = false;
    private int currentState;
    private NetworkTable visionTable;
    private boolean turnLeft = false;
    private int autonomousMode;
    public AutonomousCommand() {
        requires(Robot.driveTrain); // Using requires, we depend on subsystem `driveTrain`.
    }
    protected void initialize() {
        Robot.driveTrain.resetLeftEncoder();
        Robot.driveTrain.resetRightEncoder();
        Robot.driveTrain.setActualDriveSpeed(0.0);
    }
    protected void execute() {
        Robot.lights.getTargetingLED().set(Relay.Value.kOn);
        SmartDashboard.putNumber("Left Encoder Reading: ", Robot.driveTrain.getLeftDistance());
        if (Robot.driveTrain.getLeftDistance() > -1 * DRIVE_CORRECTION * DRIVE_FORWARD_DISTANCE || Robot.driveTrain.getRightDistance() > -1 * DRIVE_CORRECTION * DRIVE_FORWARD_DISTANCE) {
            Robot.driveTrain.teleopControl(AUTO_DRIVE_SPEED, 0);
            SmartDashboard.putNumber("Autoforward Speed: ", AUTO_DRIVE_SPEED);
        } else {
            RobotMap.driveTrainRM.arcadeDrive(0, 0);
        }
    }
    protected boolean isFinished() {
        return false;
    }
    protected void end() {
        RobotMap.driveTrainRM.arcadeDrive(0, 0);
    }
    protected void interrupted() {
        RobotMap.driveTrainRM.arcadeDrive(0, 0);
    }
}