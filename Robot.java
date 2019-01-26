package org.usfirst.frc.team3465.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.cameraserver.CameraServer;

import org.usfirst.frc.team3465.robot.MecanumDrive;
import org.usfirst.frc.team3465.robot.magEncoder;
import org.opencv.core.Mat;
//import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
//import org.opencv.imgcodecs.*;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

import com.ctre.phoenix.CTREJNIWrapper;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc.team3465.robot.UsingEncoders;

public class Robot extends TimedRobot {
	
	TalonSRX m_rearRight = new TalonSRX(3);
	TalonSRX m_frontRight = new TalonSRX(0);
	
	TalonSRX m_frontLeft = new TalonSRX(1);
	TalonSRX m_rearLeft = new TalonSRX(2);

	/*PWMTalonSRX m_rearRight1 = new PWMTalonSRX(0);
	PWMTalonSRX m_frontRight1 = new PWMTalonSRX(1);
	
	PWMTalonSRX m_frontLeft1 = new PWMTalonSRX(2);
	PWMTalonSRX m_rearLeft1 = new PWMTalonSRX(3);*/
	
	PWMTalonSRX m_clawMotor = new PWMTalonSRX(4);
	
	PWMTalonSRX m_armMotor = new PWMTalonSRX(5);

	Encoder arm = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	//Encoder claw = new Encoder(2,3, false, Encoder.EncodingType.k4X);


	MecanumDrive m_mechDrive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);
	private Timer m_timer = new Timer();
	private Joystick m_stick1 = new Joystick(0);

	final int kTimeoutMs = 30;
	
    /**
	 * If the measured travel has a discontinuity, Note the extremities or
	 * "book ends" of the travel.
	 */
	final boolean kDiscontinuityPresent = true;
	final int kBookEnd_0 = 910;		/* 80 deg */
	final int kBookEnd_1 = 1137;	/* 100 deg */

	
	@Override
	public void robotInit() {

		m_rearRight.configFactoryDefault();

		/* Seed quadrature to be absolute and continuous */
		initQuadrature();
		/* Configure Selected Sensor for Talon */
		m_rearRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,	// Feedback
											0, 											// PID ID
											kTimeoutMs);								// Timeout

		applyDeadband();


		new Thread(() -> {
         UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
         camera.setResolution(640, 480);
         
         CvSink cvSink = CameraServer.getInstance().getVideo();
         CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
         
         Mat source = new Mat();
         Mat output = new Mat();
         
         while(!Thread.interrupted()) {
             cvSink.grabFrame(source);
             Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
             outputStream.putFrame(output);
         }
	 }).start();
	 
	}


	
	
	private void applyDeadband() {
		m_clawMotor.enableDeadbandElimination(true);
		m_armMotor.enableDeadbandElimination(true);	
		//m_rearLeft.enableDeadbandElimination(true);	
	}
	      

	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
		
	}


	@Override
	public void autonomousPeriodic() {
	}
	

	@Override
	public void teleopInit() {
		

	}

	

	@Override
	public void teleopPeriodic() {
		initQuadrature();
		if(m_stick1.getRawButton(3)){
			UsingEncoders.runToPosition(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight, 36000);
		}

		if(m_stick1.getY() > 0 || m_stick1.getX() > 0|| m_stick1.getRawAxis(4) > 0){
		m_mechDrive.fwbw(m_stick1.getX(), -m_stick1.getY(), m_stick1.getRawAxis(4),  0.0, 1, 0.8);
		}

		if(m_stick1.getY() < 0 || m_stick1.getX() < 0|| m_stick1.getRawAxis(4) < 0){
		m_mechDrive.fwbw(m_stick1.getX(), -m_stick1.getY(), m_stick1.getRawAxis(4),  0.0, 1, 0.8);
   		}

		if (m_stick1.getRawButton(2)){
		m_clawMotor.set(0.1);
		}
		else{
			m_clawMotor.stopMotor();
		}
		if (m_stick1.getRawButton(1)){ //3
		m_clawMotor.set(-0.1);
		}
		else{
			m_clawMotor.stopMotor();
		}

		int selSenPos = m_rearRight.getSelectedSensorPosition(0);
		int pulseWidthWithoutOverflows = m_rearRight.getSensorCollection().getPulseWidthPosition() & 0xFFF;
 				
		/**
		 * Display how we've adjusted PWM to produce a QUAD signal that is
		 * absolute and continuous. Show in sensor units and in rotation
		 * degrees.
		 */
		DriverStation.reportError("pulseWidPos:" + pulseWidthWithoutOverflows +
						 "   =>    " + "selSenPos:" + selSenPos, false);
		DriverStation.reportError("      ", false);
		DriverStation.reportError("pulseWidDeg:" + ToDeg(pulseWidthWithoutOverflows) +
						 "   =>    " + "selSenDeg:" + ToDeg(selSenPos), false);
		System.out.println();
	}
	
	/**
	 * @param units CTRE mag encoder sensor units 
	 * @return degrees rounded to tenths.
	 */
	String ToDeg(int units) {
		double deg = units * 360.0 / 4096.0;

		/* truncate to 0.1 res */
		deg *= 10;
		deg = (int) deg;
		deg /= 10;

		return "" + deg;
	}

	
	/**
	 * Seed the quadrature position to become absolute. This routine also
	 * ensures the travel is continuous.
	 */
	public void initQuadrature() {
		/* get the absolute pulse width position */
		int pulseWidth = m_rearLeft.getSensorCollection().getPulseWidthPosition();

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

		/* Update Quadrature position */
		m_rearLeft.getSensorCollection().setQuadraturePosition(pulseWidth, kTimeoutMs);
	}
	
	
	@Override
	public void testPeriodic() {
	}
}
