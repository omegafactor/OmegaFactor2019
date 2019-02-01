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


public class EncodersVelocity {

    public static double averageVelocity(TalonSRX frontLeft, TalonSRX frontRight, TalonSRX rearLeft, TalonSRX rearRight){
        double frontLeftVelocity = Math.abs(frontLeft.getSelectedSensorVelocity());
        double frontRightVelocity = Math.abs(frontRight.getSelectedSensorVelocity());
        double rearLeftVelocity = Math.abs(rearLeft.getSelectedSensorVelocity());
        double rearRightVelocity = Math.abs(rearRight.getSelectedSensorVelocity());
        double averageVelocity = (frontLeftVelocity + frontRightVelocity + rearLeftVelocity + rearRightVelocity) / 4;
        return averageVelocity;
    }

    public static double frontLeftVelocity(TalonSRX frontLeft){
        double frontLeftVelocity = Math.abs(frontLeft.getSelectedSensorVelocity());
        return frontLeftVelocity;
    }

    public static double frontRightVelocity(TalonSRX frontRight){
        double frontRightVelocity = Math.abs(frontRight.getSelectedSensorVelocity());
        return frontRightVelocity;
    }

    public static double rearLeftVelocity(TalonSRX rearLeft){
        double rearLeftVelocity = Math.abs(rearLeft.getSelectedSensorVelocity());
        return rearLeftVelocity;
    }

    public static double rearRightVelocity(TalonSRX rearRight){
        double rearRightVelocity = Math.abs(rearRight.getSelectedSensorVelocity());
        return rearRightVelocity;
    }

    


    
}
