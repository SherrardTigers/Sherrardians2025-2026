package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

import java.util.Locale;

@TeleOp(name = "31893 Competition w/ Auto-Aim")
public class camtest extends LinearOpMode {

  private DcMotor motor1;
  private DcMotor motor2;
  private DcMotor motor3;
  private DcMotor motor4;
  private DcMotor speedyleft;
  private DcMotor intake2;
  private DcMotor speedyright;
  private DcMotor intake1;
  private CRServo servolaunch;
  private CRServo servo1;

  GoBildaPinpointDriver odo;
  double oldTime = 0;
  
  // Voltage compensation parameters
  private static final double NOMINAL_VOLTAGE = 13.5;
  private static final double BASE_LAUNCHER_POWER = 0.46;
  
  // Vision
  private VisionPortal visionPortal;
  private AprilTagProcessor aprilTag;
  
  // AprilTag IDs
  private static final int BLUE_TAG = 20;
  private static final int RED_TAG = 24;
  
  // Auto-aim parameters - IMPROVED TUNING
  private static final double AIM_TURN_SPEED = 0.3;  // Increased max speed
  private static final double TOLERANCE_DEGREES = 1;  // Wider tolerance to prevent oscillation
  private static final double MIN_TURN_POWER = 0.25;  // Minimum power to overcome friction
  private static final double PROPORTIONAL_GAIN = 0.025;  // Proportional control gain

  @Override
  public void runOpMode() {
    ElapsedTime runtime;
    float axial;
    float lateral;
    float yaw;
    float frontLeftPower;
    float frontRightPower;
    float backLeftPower;
    float backRightPower;
    double max;

    motor1 = hardwareMap.get(DcMotor.class, "motor 1");
    motor2 = hardwareMap.get(DcMotor.class, "motor 2");
    motor3 = hardwareMap.get(DcMotor.class, "motor 3");
    motor4 = hardwareMap.get(DcMotor.class, "motor 4");
    speedyleft = hardwareMap.get(DcMotor.class, "speedy left");
    intake2 = hardwareMap.get(DcMotor.class, "intake 2");
    speedyright = hardwareMap.get(DcMotor.class, "speedy right");
    intake1 = hardwareMap.get(DcMotor.class, "intake 1");
    servolaunch = hardwareMap.get(CRServo.class, "servo launch");
    servo1 = hardwareMap.get(CRServo.class, "servo1");

    runtime = new ElapsedTime();
    
    motor1.setDirection(DcMotor.Direction.FORWARD);
    motor2.setDirection(DcMotor.Direction.FORWARD);
    motor3.setDirection(DcMotor.Direction.REVERSE);
    motor4.setDirection(DcMotor.Direction.REVERSE);
    speedyright.setDirection(DcMotor.Direction.REVERSE);
    intake2.setDirection(DcMotor.Direction.REVERSE);
    gamepad1.setLedColor(255, 0.5, 0, 10000000);
    gamepad2.setLedColor(1, 0, 1, 10000000);
    
    double speedMultiplyer = .6;
    double movementSpeed = 1;
    
    // Initialize odometry
    odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
    odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
    odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    odo.resetPosAndIMU();
    
    // Initialize AprilTag detection
    aprilTag = new AprilTagProcessor.Builder()
        .build();
    
    // Initialize camera
    visionPortal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .addProcessor(aprilTag)
        .build();

    telemetry.addData("Status", "Initialized");
    telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
    telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
    telemetry.addData("Device Version Number:", odo.getDeviceVersion());
    telemetry.addData("Heading Scalar", odo.getYawScalar());
    telemetry.addData("Camera", "AprilTag detection ready");
    telemetry.addData("Auto-Aim", "DPAD LEFT=Blue, RIGHT=Red");
    telemetry.update();
    
    waitForStart();
    runtime.reset();
    
    while (opModeIsActive()) {
      // Get AprilTag detections
      List<AprilTagDetection> detections = aprilTag.getDetections();
      
      // Drive controls
      axial = -gamepad1.left_stick_y;
      lateral = gamepad1.left_stick_x;
      yaw = gamepad1.right_stick_x;
      
      // AUTO-AIM OVERRIDE: Check if DPAD LEFT or RIGHT is pressed
      boolean autoAiming = false;
      if (gamepad1.dpad_left) {
        // Auto-aim to Blue target (Tag 20)
        AprilTagDetection blueTarget = null;
        for (AprilTagDetection detection : detections) {
          if (detection.id == BLUE_TAG) {
            blueTarget = detection;
            break;
          }
        }
        
        if (blueTarget != null) {
          double bearing = blueTarget.ftcPose.bearing;
          
          if (Math.abs(bearing) > TOLERANCE_DEGREES) {
            // Improved proportional control with deadband compensation
            double turnPower = Math.abs(bearing) * PROPORTIONAL_GAIN;
            // Clamp between minimum and maximum, add deadband compensation
            turnPower = Math.max(MIN_TURN_POWER, Math.min(AIM_TURN_SPEED, turnPower));
            yaw = (float) (bearing > 0 ? -turnPower : turnPower);
            autoAiming = true;
            telemetry.addData("🔵 BLUE Auto-Aim", "Adjusting... %.1f° off", bearing);
          } else {
            yaw = 0;
            autoAiming = true;
            telemetry.addData("🔵 BLUE Auto-Aim", "🎯 LOCKED ON!");
          }
          telemetry.addData("Target", "BLUE (Tag 20)");
          telemetry.addData("Distance", "%.1f inches", blueTarget.ftcPose.range);
          telemetry.addData("Bearing", "%.2f°", bearing);
        } else {
          telemetry.addData("🔵 BLUE Auto-Aim", "⚠ Tag 20 not visible!");
        }
      }
      else if (gamepad1.dpad_right) {
        // Auto-aim to Red target (Tag 24)
        AprilTagDetection redTarget = null;
        for (AprilTagDetection detection : detections) {
          if (detection.id == RED_TAG) {
            redTarget = detection;
            break;
          }
        }
        
        if (redTarget != null) {
          double bearing = redTarget.ftcPose.bearing;
          
          if (Math.abs(bearing) > TOLERANCE_DEGREES) {
            // Improved proportional control with deadband compensation
            double turnPower = Math.abs(bearing) * PROPORTIONAL_GAIN;
            // Clamp between minimum and maximum, add deadband compensation
            turnPower = Math.max(MIN_TURN_POWER, Math.min(AIM_TURN_SPEED, turnPower));
            yaw = (float) (bearing > 0 ? -turnPower : turnPower);
            autoAiming = true;
            telemetry.addData("🔴 RED Auto-Aim", "Adjusting... %.1f° off", bearing);
          } else {
            yaw = 0;
            autoAiming = true;
            telemetry.addData("🔴 RED Auto-Aim", "🎯 LOCKED ON!");
          }
          telemetry.addData("Target", "RED (Tag 24)");
          telemetry.addData("Distance", "%.1f inches", redTarget.ftcPose.range);
          telemetry.addData("Bearing", "%.2f°", bearing);
        } else {
          telemetry.addData("🔴 RED Auto-Aim", "⚠ Tag 24 not visible!");
        }
      }
      
      // Calculate motor powers
      frontLeftPower = axial + lateral + yaw;
      frontRightPower = (axial - lateral) - yaw;
      backLeftPower = (axial - lateral) + yaw;
      backRightPower = (axial + lateral) - yaw;
      
      max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));
      if (max > 1) {
        frontLeftPower = (float) (frontLeftPower / max);
        frontRightPower = (float) (frontRightPower / max);
        backLeftPower = (float) (backLeftPower / max);
        backRightPower = (float) (backRightPower / max);
      }
      
      motor1.setPower(frontLeftPower*movementSpeed);
      motor3.setPower(frontRightPower*movementSpeed);
      motor2.setPower(backLeftPower*movementSpeed);
      motor4.setPower(backRightPower*movementSpeed);
      
      // VOLTAGE-COMPENSATED LAUNCHER CONTROL
      double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
      double voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;
      double rawPower = gamepad1.left_trigger * speedMultiplyer;
      double compensatedPower = rawPower * voltageCompensation;
      compensatedPower = Math.min(1.0, Math.max(0.0, compensatedPower));
      
      speedyleft.setPower(compensatedPower);
      speedyright.setPower(compensatedPower);
      
      // Intake controls
      intake1.setPower(gamepad2.left_trigger);
      intake2.setPower(gamepad2.left_trigger);
      intake1.setPower(gamepad1.right_trigger);
      intake2.setPower(gamepad1.right_trigger);
      
      // Servo controls
      if (gamepad1.a || gamepad2.a) {
        servolaunch.setPower(1);
        servo1.setPower(1);
      } else if (gamepad1.b || gamepad2.b) {
        servolaunch.setPower(-1);
        servo1.setPower(-1);
      } else {
        servolaunch.setPower(0);
        servo1.setPower(0);
      }
      
      // Speed adjustment (only dpad_up/down for speed, left/right for aiming)
      if (gamepad1.dpad_up && !autoAiming) {
        movementSpeed = 1;
      } else if (gamepad1.dpad_down && !autoAiming) {
        movementSpeed = .50;
      }
      
      if (gamepad2.dpad_down) {
        speedMultiplyer = .49;
      } else if (gamepad2.dpad_up) {
        speedMultiplyer = .594;
      }
      
      // Odometry update
      odo.update();
      
      double newTime = getRuntime();
      double loopTime = newTime-oldTime;
      double frequency = 1/loopTime;
      oldTime = newTime;
      
      Pose2D pos = odo.getPosition();
      String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
      double heading = pos.getHeading(AngleUnit.DEGREES);
      telemetry.addData("Position", data);
      
      double longheading = Math.abs(pos.getHeading(AngleUnit.DEGREES) - 20);
      double shortheading = Math.abs(pos.getHeading(AngleUnit.DEGREES) - 45);
      
      String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
      telemetry.addData("Velocity", velocity);
      telemetry.addData("Status", odo.getDeviceStatus());
      telemetry.addData("Pinpoint Frequency", odo.getFrequency());
      telemetry.addData("REV Hub Frequency: ", frequency);
      
      if (longheading < 10) {
        telemetry.addData("Longshot", "ready");
      } 
      if (shortheading < 10) {
        telemetry.addData("Shortshot", "ready");
      }
      
      // VOLTAGE TELEMETRY
      telemetry.addData("Battery Voltage", "%.2f V", currentVoltage);
      telemetry.addData("Launcher Power (Compensated)", "%.2f", compensatedPower);
      
      telemetry.addData("Status", "Run Time: " + runtime);
      telemetry.addData("Front left/Right", JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(frontRightPower, 4, 2));
      telemetry.addData("Back  left/Right", JavaUtil.formatNumber(backLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(backRightPower, 4, 2));
      telemetry.update();
    }
    
    visionPortal.close();
  }
}