/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.cameraserver.CameraServer;

import frc.robot.MecanumDrive;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;

//import org.usfirst.frc.team3465.robot.UsingEncoders;

public class Robot extends TimedRobot {

	TalonSRX m_rearRight = new TalonSRX(3);
	TalonSRX m_frontRight = new TalonSRX(0);
	TalonSRX m_frontLeft = new TalonSRX(1);
	TalonSRX m_rearLeft = new TalonSRX(2);

	PWMTalonSRX m_clawMotor = new PWMTalonSRX(0);
	
	TalonSRX m_armMotor = new TalonSRX(4);

	
	MecanumDrive m_mechDrive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);
	private Timer m_timer = new Timer();
  private Joystick m_stick1 = new Joystick(0);
  private Joystick m_stick2 = new Joystick(1);

	@Override
	public void robotInit() {
		m_rearRight.configFactoryDefault();
		m_rearLeft.configFactoryDefault();
		m_frontLeft.configFactoryDefault();
		m_frontRight.configFactoryDefault();

		applyDeadband();

        
    
    UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture();
    camera1.setResolution(640, 480);
        
    UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture();
    camera2.setResolution(640, 480);
	 
	}
	

	private void applyDeadband() {
		m_clawMotor.enableDeadbandElimination(true);
	}
	      

	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
		
	}


	@Override
	public void autonomousPeriodic() {
		if(m_stick1.getY() > 0 || m_stick1.getX() > 0 || m_stick1.getRawAxis(4) > 0){
			m_mechDrive.fwbw(m_stick1.getX(), -m_stick1.getY(), m_stick1.getRawAxis(4),  0.0, 1, .8);
			}
	
			if(m_stick1.getY() < 0 || m_stick1.getX() < 0 || m_stick1.getRawAxis(4) < 0){
			m_mechDrive.fwbw(m_stick1.getX(), -m_stick1.getY(), m_stick1.getRawAxis(4),  0.0, 1, .8);
				 }
	
			if (m_stick2.getRawButton(1)){
				m_armMotor.set(ControlMode.PercentOutput, -1);
			}else if (m_stick2.getRawButton(4)){
				m_armMotor.set(ControlMode.PercentOutput, 1);
			}
			else{
				m_armMotor.set(ControlMode.PercentOutput, 0);
			}
			
	
			if(m_stick2.getRawButton(5)){
				m_clawMotor.set(-.3);
			}else if(m_stick2.getRawButton(6)){
				m_clawMotor.set(.45);
			}else{
				m_clawMotor.set(0);
			}
	}
	

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {

	
		if(m_stick1.getY() > 0 || m_stick1.getX() > 0 || m_stick1.getRawAxis(4) > 0){
		m_mechDrive.fwbw(m_stick1.getX(), -m_stick1.getY(), m_stick1.getRawAxis(4),  0.0, 1, .8);
		}

		if(m_stick1.getY() < 0 || m_stick1.getX() < 0 || m_stick1.getRawAxis(4) < 0){
		m_mechDrive.fwbw(m_stick1.getX(), -m_stick1.getY(), m_stick1.getRawAxis(4),  0.0, 1, .8);
   		}

		if (m_stick2.getRawButton(1)){
			m_armMotor.set(ControlMode.PercentOutput, -1);
		}else if (m_stick2.getRawButton(4)){
			m_armMotor.set(ControlMode.PercentOutput, 1);
		}
		else{
			m_armMotor.set(ControlMode.PercentOutput, 0);
		}
		

		if(m_stick2.getRawButton(5)){
			m_clawMotor.set(-.3);
		}else if(m_stick2.getRawButton(6)){
			m_clawMotor.set(.3);
		}else{
			m_clawMotor.set(0);
		}

		
		
	}
	

	@Override
	public void testPeriodic() {
	}
}
