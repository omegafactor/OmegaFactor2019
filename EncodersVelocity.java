/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3465.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.ControlMode;


public class EncodersVelocity {

    public static double averageVelocity(TalonSRX frontLeft, TalonSRX rearRight){
        double frontLeftVelocity = Math.abs(frontLeft.getSelectedSensorVelocity());
        double rearRightVelocity = Math.abs(rearRight.getSelectedSensorVelocity());
        double averageVelocity = (frontLeftVelocity + rearRightVelocity) / 2;
        return averageVelocity;
    }

    
    public static double frontLeftVelocity(TalonSRX frontLeft){
        double frontLeftVelocity = Math.abs(frontLeft.getSelectedSensorVelocity());
        return frontLeftVelocity;
    }

    public static double rearRightVelocity(TalonSRX rearRight){
        double rearRightVelocity = Math.abs(rearRight.getSelectedSensorVelocity());
        return rearRightVelocity;
    }

    


    
}
