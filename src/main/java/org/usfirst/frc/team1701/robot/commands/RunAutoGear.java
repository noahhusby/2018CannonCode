/**
 * commands/RunAutoGear.java
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
import edu.wpi.first.wpilibj.networktables.NetworkTable;
public class RunAutoGear extends Command {
	private NetworkTable visionTable;
	private boolean gearTargetFound;
	private boolean gearTargetLinedUp;
	private boolean passesHWTest;
	private double gearTargetX;
	private double gearTargetHWError;
	private double oldHWError;
	private boolean finished;
	private int currentState;
	private double distanceToCorrect = 0;
	private double degreesToTurn;
	private boolean turnLeft = true;
	private final double TURN_SPEED = .3;
	private final double DRIVE_SPEED = .7;
	public RunAutoGear() {
		requires(Robot.driveTrain);
		requires(Robot.gearArm);
	}
	protected void initialize() {
		NetworkTable.setTeam(1701);
		visionTable = NetworkTable.getTable("vision");
		finished = false;
		currentState = 1;
	}
	protected void execute() {
		try {
			gearTargetFound = visionTable.getBoolean("gearTargetFound", false);
			gearTargetLinedUp = visionTable.getBoolean("gearTargetLinedUp", false);
			passesHWTest = visionTable.getBoolean("passesHWTest", false);
			gearTargetX = visionTable.getNumber("gearTargetX", -1);
			gearTargetHWError = visionTable.getNumber("gearTargetHWError", 0);
		} catch(Exception e) {
			e.printStackTrace();
		}
		// Turn on lights if gear target found.
		if(gearTargetFound) {
			if(RobotMap.lightsLED2.get() == Relay.Value.kReverse) {
				RobotMap.lightsLED2.set(Relay.Value.kOn);
			} else {
				RobotMap.lightsLED2.set(Relay.Value.kForward);
			}
		}
		switch (currentState) {
			// different actions depending on the state the gear placement is in
			case 0: // DONE
				finished = true;
				break;
			case 1: // SEARCHING
				if (gearTargetFound) {
					/*
					 * if the vision code recognizes a valid gear target either it
					 * sees a pair of two rectangles that fulfill criteria or just
					 * one rectangle. cannot reliably test if the singular rectangle
					 * is valid, so if there is only one it will automagically
					 * register as valid
					 */
					if (gearTargetLinedUp) {
						/*
						 * if the rightmost end of the rightmost rectangle is in the
						 * center of the robot's vision. this is what it should be
						 * like to put the gear on. if there is only one rectangle,
						 * it uses that rectangle's right side
						 */
						currentState++;
					} else if (passesHWTest) {
						double distanceToWall = Robot.gearArm.getGearDistance(); // in
																					// inches
						double degreesOff = .09375 * (gearTargetX - 320);
						distanceToCorrect = distanceToWall * Math.tan(degreesOff); // in
																					// inches
						RobotMap.navx.reset();
						degreesToTurn = 90;
						currentState = 4;
					} else {
						if (gearTargetHWError < oldHWError) {
							/*
							 * If this is more accurate than before, keep turning in
							 * the same direction as before, at a speed proportional
							 * to the percent error of the H:W test
							 */
							turn(gearTargetHWError / 100);
						} else if (gearTargetHWError > oldHWError) {
							/*
							 * If this is less accurate than before, turn in the
							 * opposite direction as of before, at a speed
							 * proportional to the percent error of the H:W test
							 */
							if (turnLeft)
								turnLeft = false;
							else
								turnLeft = true;
							turn(gearTargetHWError / 100);
						} else {
							/*
							 * If this is just as accurate as before, turn in the
							 * opposite direction as of before, at a speed
							 * proportional to half the percent error of the H:W
							 * test
							 */
							if (turnLeft)
								turnLeft = false;
							else
								turnLeft = true;
							turn(.5 * gearTargetHWError / 100);
						}
					}	
				}
				break;
			case 2: // LOCKED-ON
				double distance = 5;
				if (Robot.gearArm.getGearDistance() > distance) {
					Robot.driveTrain.leftDriveControl(DRIVE_SPEED);
					Robot.driveTrain.rightDriveControl(DRIVE_SPEED);
				} else {
					currentState++;
				}
				break;
			case 3: // PLACING
				// Robot.gearAzrm.pushGear();
				currentState = 0;
				break;
			case 4: // REVERSE 1
				currentState++;
				// reversing away from the wall if necessary
				// currently does not reverse at all
				break;
			case 5: // TURN 1
				if (degreesToTurn > Math.abs(RobotMap.navx.getYaw())) {
					if (distanceToCorrect > 0)
						turnLeft = true;
					else
						turnLeft = false;
					turn(TURN_SPEED);
				} else {
					Robot.driveTrain.resetLeftEncoder();
					Robot.driveTrain.resetRightEncoder();
					currentState++;
				}
				break;
			case 6: // REVERSE 2
				if (Robot.driveTrain.getLeftDistance() < distanceToCorrect
						|| Robot.driveTrain.getRightDistance() < distanceToCorrect) {
					Robot.driveTrain.leftDriveControl(-DRIVE_SPEED);
					Robot.driveTrain.rightDriveControl(-DRIVE_SPEED);
				} else {
					RobotMap.navx.reset();
					degreesToTurn = 90;
					currentState++;
				}
				break;
			case 7: // TURN 2
				if (degreesToTurn > Math.abs(RobotMap.navx.getYaw())) {
					if (distanceToCorrect > 0)
						turnLeft = false;
					else
						turnLeft = true;
					turn(TURN_SPEED);
				} else {
					currentState = 1;
				}
				break;
		}
		oldHWError = gearTargetHWError;
	}
	private void turn(double speed) {
		if(turnLeft) {
			Robot.driveTrain.leftDriveControl(speed);
			Robot.driveTrain.rightDriveControl(-speed);
		} else {
			Robot.driveTrain.leftDriveControl(-speed);
			Robot.driveTrain.rightDriveControl(speed);
		}
	}
	protected boolean isFinished() {
		return finished;
	}
	protected void end() {}
	protected void interrupted() {}
}