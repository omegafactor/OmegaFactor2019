/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3465.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Joystick;
/**
 * Add your docs here.
 */
public class UsingEncoders {

    final boolean kDiscontinuityPresent = true;
	final int kBookEnd_0 = 910;		/* 80 deg */
	final int kBookEnd_1 = 1137;	/* 100 deg */
    final int kTimeoutMs = 30;
	
	
    public static void runToPosition(TalonSRX rearRight, int Degrees){
		Joystick m_stick = new Joystick(0);
		if(m_stick.getRawButton(3)){
		rearRight.setSelectedSensorPosition(0);
        double selSenPos = ToDeg(rearRight.getSelectedSensorPosition(0));
		double runToDegrees = selSenPos + Degrees;


		rearRight.set(ControlMode.PercentOutput,.5);
		while(selSenPos <= runToDegrees){
			selSenPos = ToDeg(rearRight.getSelectedSensorPosition(0));
		}
		
		rearRight.set(ControlMode.PercentOutput,0);
	}
    }
    public static double ToDeg(int units) {
		double deg = units * 360.0 / 4096.0;

		/* truncate to 0.1 res */
		deg *= 10;
		deg = (int) deg;
		deg /= 10;

		return deg;
    }
}
