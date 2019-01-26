/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3465.robot;

/**
 * Add your docs here.
 */
import edu.wpi.first.wpilibj.AnalogInput;
public class UltrasonicMeasurements {
    AnalogInput ultra = new AnalogInput(0);

    public double getDistanceMm() {
        double averageVoltage = ultra.getAverageVoltage();
        //double voltageIn = 5 / 1024d;
        double mmDistance = 5 * averageVoltage / (4.959 / 1024d);
        //Log.i("Distance: " + mmDistance + "; " + averageVoltage);
        return mmDistance;
    }

    public double getDistanceCm() {
        double centimeters = getDistanceMm() / 10;
        return centimeters;
    }

    public double getDistanceIn() {
        double inches = getDistanceCm() / 2.54;
        return inches;
    }

}
