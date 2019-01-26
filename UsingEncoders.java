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

/**
 * Add your docs here.
 */
public class UsingEncoders {

    final boolean kDiscontinuityPresent = true;
	final int kBookEnd_0 = 910;		/* 80 deg */
	final int kBookEnd_1 = 1137;	/* 100 deg */
    final int kTimeoutMs = 30;

    public void initQuadrature(TalonSRX frontLeft, TalonSRX frontRight, TalonSRX rearLeft, TalonSRX rearRight, double Degrees){
		/* get the absolute pulse width position */
		int pulseWidth = rearRight.getSensorCollection().getPulseWidthPosition();

		/**
		 * If there is a discontinuity in our measured range, subtract one half
		 * rotation to remove it
		 */
		if (kDiscontinuityPresent) {

			/* Calculate the center */
			int newCenter;
			newCenter = (kBookEnd_0 + kBookEnd_1) / 2;
			newCenter &= 0xFFF;

			/**
			 * Apply the offset so the discontinuity is in the unused portion of
			 * the sensor
			 */
			pulseWidth -= newCenter;
		}

		/**
		 * Mask out the bottom 12 bits to normalize to [0,4095],
		 * or in other words, to stay within [0,360) degrees 
		 */
		pulseWidth = pulseWidth & 0xFFF;

    }
    
    public static void runToPosition(TalonSRX frontLeft, TalonSRX frontRight, TalonSRX rearLeft, TalonSRX rearRight, int Degrees){
        double selSenPos = ToDeg(rearRight.getSelectedSensorPosition(0));
        double initialDegree = selSenPos;
        double runToDegrees = initialDegree + Degrees;
		//int pulseWidthWithoutOverflows = rearRight.getSensorCollection().getPulseWidthPosition() & 0xFFF;
        
        frontLeft.set(ControlMode.PercentOutput,.5);
        frontRight.set(ControlMode.PercentOutput,-.5);
        rearLeft.set(ControlMode.PercentOutput,.5);
        rearRight.set(ControlMode.PercentOutput,-.5);
        while(selSenPos <= runToDegrees){
            //selSenPos = ToDeg(rearRight.getSelectedSensorPosition(0));
        }
        frontLeft.set(ControlMode.PercentOutput,0);
        frontRight.set(ControlMode.PercentOutput,0);
        rearLeft.set(ControlMode.PercentOutput,0);
        rearRight.set(ControlMode.PercentOutput,0);
        
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
