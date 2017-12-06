/**
 * commands/TeleopDrive.java
 *
 * Created by Nicholas Hubbard on 11/10/2017.
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class TeleopDrive extends Command {
	public TeleopDrive() {
		requires(Robot.driveTrain);
	}
	protected void initialize() {
		Robot.driveTrain.resetLeftEncoder();
		Robot.driveTrain.resetRightEncoder();
		Robot.driveTrain.preparePIDForTeleop();
	}
	protected void execute() {
		Robot.lights.getTargetingLED().set(Relay.Value.kOn);
		SmartDashboard.putNumber("Left Encoder Reading: ", Robot.driveTrain.getLeftDistance());
		SmartDashboard.putNumber("Right Encoder Reading: ", Robot.driveTrain.getRightDistance());
		SmartDashboard.putNumber("Navx Reading: ", RobotMap.navx.getYaw());
		double deadConst = .10;
		double fBInput = checkDeadZone(Robot.oi.drive_FB.getY(), deadConst);
		double tInput = .75 * checkDeadZone(Robot.oi.drive_T.getX(), deadConst);
		Robot.driveTrain.teleopControl(fBInput, tInput);
	}
	protected boolean isFinished() {
		return false;
	}
	protected void end() {}
	protected void interrupted() {}
	private double checkDeadZone(double input, double deadConst) {
		if(input > 0) {
			if(deadConst >= input) {
				input = 0;
			}
		} else {
			if(-deadConst <= input) {
				input = 0;
			}
		}
		return input;
	}
}