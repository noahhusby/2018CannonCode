/**
 * subsystems/DriveTrain.java
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
package org.usfirst.frc.team1701.robot.subsystems;
import org.usfirst.frc.team1701.robot.Robot;
import org.usfirst.frc.team1701.robot.RobotMap;
import org.usfirst.frc.team1701.robot.commands.AutonomousCommand;
import org.usfirst.frc.team1701.robot.commands.TeleopDrive;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
public class DriveTrain extends Subsystem implements PIDOutput {
	private final SpeedController left_1 = RobotMap.driveTrainLeft_1;
	private final SpeedController left_2 = RobotMap.driveTrainLeft_2;
	private final SpeedController right_1 = RobotMap.driveTrainRight_1;
	private final SpeedController right_2 = RobotMap.driveTrainRight_2;
	private boolean reversed = true;
	private final double DIST_ADJUST_CONST = 1052.6;
	private boolean precise = false;
	private PIDController pid;
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;
	static final double kToleranceDegrees = 2.0;
	private double rate;
	private double startAngle;
	private double actualDriveSpeed;
	private final boolean STRAIGHTEN_WITH_PID = false;
	public double getActualDriveSpeed() {
		return actualDriveSpeed;
	}
	public void setActualDriveSpeed(double actualDriveSpeed) {
		this.actualDriveSpeed = actualDriveSpeed;
	}
	public double getStartAngle() {
		return startAngle;
	}
	public void setupPID() {
		pid = new PIDController(kP, kI, kD, kF, RobotMap.navx, this);
		pid.setInputRange(-180, 180);
		pid.setOutputRange(-.5, .5);
		pid.setAbsoluteTolerance(kToleranceDegrees);
		pid.setContinuous(true);
		pid.enable();
	}
	public void setPIDtoAngle(double startAngle) {
		this.startAngle = startAngle;
		pid.setSetpoint(startAngle);
	}
	public boolean isPIDnull() {
		return (pid == null);
	}
	public boolean isPrecise() {
		return precise;
	}
	public void setPrecise(boolean precise) {
		this.precise = precise;
	}
	private final CANTalon leftEncTalon = (CANTalon) left_2;
	private final CANTalon rightEncTalon = (CANTalon) right_1;
	private final double WHEEL_CIRCUMFERENCE = 3.9 * Math.PI;
	private final int PULSES_PER_ROTATION = 1440;
	public void setup() {
		leftEncTalon.configEncoderCodesPerRev(PULSES_PER_ROTATION);
		rightEncTalon.configEncoderCodesPerRev(PULSES_PER_ROTATION);
	}
	public void enablePID() {
		pid.enable();
	}
	public void disablePID() {
		pid.disable();
	}
	public int getLeftVelocity() {
		return leftEncTalon.getEncVelocity();
	}
	public double getLeftDistance() {
		return leftEncTalon.getEncPosition() * WHEEL_CIRCUMFERENCE / DIST_ADJUST_CONST;
	}
	public void resetLeftEncoder() {
		leftEncTalon.setEncPosition(0);
	}
	public int getRightVelocity() {
		return rightEncTalon.getEncVelocity();
	}
	public double getRightDistance() {
		return -rightEncTalon.getEncPosition() * WHEEL_CIRCUMFERENCE / DIST_ADJUST_CONST;
	}
	public void resetRightEncoder() {
		rightEncTalon.setEncPosition(0);
	}
	public void toggleReversed() {
		if (reversed) {
			reversed = false;
		} else {
			reversed = true;
		}
	}
	public void preparePIDForTeleop() {
		return;
	}
	public void leftDriveControl(double inputSpeed) {
		left_1.set(inputSpeed);
		left_2.set(inputSpeed);
	}
	public void rightDriveControl(double inputSpeed) {
		right_1.set(inputSpeed);
		right_2.set(inputSpeed);
	}
	public void teleopControl(double forwardsBackwardsAxis, double turningAxis) {
		if (reversed) {
			forwardsBackwardsAxis *= -1;
		}

		if (precise) {
			forwardsBackwardsAxis *= .5;
			turningAxis *= .75;
		}
		RobotMap.driveTrainRM.arcadeDrive(forwardsBackwardsAxis, turningAxis);
	}
	public void initDefaultCommand() {
		setDefaultCommand(new TeleopDrive());
	}
	public void driveBackwardsPID() {
		if (Math.abs(actualDriveSpeed) < Math.abs(AutonomousCommand.AUTO_DRIVE_SPEED)) {
			actualDriveSpeed += .03;
		}
		RobotMap.driveTrainRM.arcadeDrive(actualDriveSpeed, rate);
	}
	@Override
	public void pidWrite(double output) {
		rate = output;
	}
}