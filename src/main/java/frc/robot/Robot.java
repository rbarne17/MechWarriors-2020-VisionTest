/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Spark m_leftSpark = new Spark(0); // Left motor controller
  private final Spark m_rightSpark = new Spark(1); // Right motor controller
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSpark, m_rightSpark); // Combined motor
                                                                                                   // control
  private final Joystick m_stick = new Joystick(0); // Primary joystick/controller
  private Ultrasonic ultrasonic = new Ultrasonic(1, 0); // Ultrasonic sensor

  private static final int IMG_WIDTH = 160; // Vision image width(pixels)
  private static final int IMG_HEIGHT = 120; // Vision image height(pixels)

  private VisionThread visionThread; // Java Thread class housing vision processing callback (init below)
  private double centerX = 1024.0; // Stores the current center of target; 1024.0 signifies "No Target"

  private final Object imgLock = new Object(); // Syncronizes read/write calls from/to centerX

  private final double visionPrecision = 0.05; // The lower this value, the more precise vision is. The higher the
                                               // value, the less the robot "jitters" trying to find the "perfect"
                                               // center
  private final double visionMinimumSpeed = 0.5; // The minimum speed at which the motors will start moving.

  /* Initializes the robot */
  @Override
  public void robotInit() {
    Thread visionThread = new Thread(() -> {
      // Get the UsbCamera from CameraServer
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      // Set the resolution
      camera.setResolution(640, 480);

      // Get a CvSink. This will capture Mats from the camera
      CvSink cvSink = CameraServer.getInstance().getVideo();
      // Setup a CvSource. This will send images back to the Dashboard
      CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

      // Mats are very memory expensive. Lets reuse this Mat.
      Mat mat = new Mat();

      // This cannot be 'true'. The program will never exit if it is. This
      // lets the robot stop this thread when restarting robot code or
      // deploying.-
      while (!Thread.interrupted()) {
        // Tell the CvSink to grab a frame from the camera and put it
        // in the source mat. If there is an error notify the output.
        if (cvSink.grabFrame(mat) == 0) {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          // skip the rest of the current iteration
          continue;
        }
        // Put a rectangle on the image
        Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 0, 255), 5);
        // Give the output stream a new image to display
        outputStream.putFrame(mat);
      }
    });
    visionThread.setDaemon(true);
    visionThread.start();
  }
}
