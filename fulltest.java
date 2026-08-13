package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

//inport odometry try test
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
// inport gobilda pinpoint driver
import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import java.util.Locale;
//stop inport

@TeleOp(name = "31893 competition")
public class fulltest extends LinearOpMode {

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


    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

  /**
   * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
   * This code will work with either a Mecanum-Drive or an X-Drive train.
   * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
   *
   * Also note that it is critical to set the correct rotation direction for each motor. See details below.
   *
   * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
   * Each motion axis is controlled by one Joystick axis.
   *
   * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
   * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
   * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
   *
   * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
   * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
   * the direction of all 4 motors (see code below).
   */
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
    // ########################################################################################
    // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
    // ########################################################################################
    //
    // Most robots need the motors on one side to be reversed to drive forward.
    // The motor reversals shown here are for a "direct drive" robot
    // (the wheels turn the same direction as the motor shaft).
    //
    // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
    // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
    // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
    //
    // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
    // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
    // <--- Click blue icon to see important note re. testing motor directions.
    motor1.setDirection(DcMotor.Direction.FORWARD);
    motor2.setDirection(DcMotor.Direction.FORWARD);
    motor3.setDirection(DcMotor.Direction.REVERSE);
    motor4.setDirection(DcMotor.Direction.REVERSE);
    speedyleft.setDirection(DcMotor.Direction.REVERSE);
    intake2.setDirection(DcMotor.Direction.REVERSE);
    gamepad1.setLedColor(255, 0.5, 0, 10000000);
    gamepad2.setLedColor(1, 0, 1, 10000000);
    
    double speedMultiplyer = .6;
    double movementSpeed = 1;
    //region odo
    // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();
    //endregion
    
    // Wait for the game to start (driver presses START)
    telemetry.addData("Status", "Initialized");
    telemetry.update();
    waitForStart();
    runtime.reset();
    // Run until the end of the match (driver presses STOP)
    
    
    while (opModeIsActive()) {
      // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
      // Note: pushing stick forward gives negative value
      axial = -gamepad1.left_stick_y;
      lateral = gamepad1.left_stick_x;
      yaw = gamepad1.right_stick_x;
      // Combine the joystick requests for each axis-motion to determine each wheel's power.
      // Set up a variable for each drive wheel to save the power level for telemetry.
      frontLeftPower = axial + lateral + yaw;
      frontRightPower = (axial - lateral) - yaw;
      backLeftPower = (axial - lateral) + yaw;
      backRightPower = (axial + lateral) - yaw;
      // Normalize the values so no wheel power exceeds 100%
      // This ensures that the robot maintains the desired motion.
      max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));
      if (max > 1) {
        frontLeftPower = (float) (frontLeftPower / max);
        frontRightPower = (float) (frontRightPower / max);
        backLeftPower = (float) (backLeftPower / max);
        backRightPower = (float) (backRightPower / max);
      }
      // Send calculated power to wheels.
      motor1.setPower(frontLeftPower*movementSpeed);
      motor3.setPower(frontRightPower*movementSpeed);
      motor2.setPower(backLeftPower*movementSpeed);
      motor4.setPower(backRightPower*movementSpeed);
      
      speedyleft.setPower(gamepad2.right_trigger * speedMultiplyer );
      speedyright.setPower(gamepad2.right_trigger * speedMultiplyer );
      intake1.setPower(gamepad2.left_trigger);
      intake2.setPower(gamepad2.left_trigger);
      //if (gamepad2.right_trigger>0) {
        //intake1.setPower(0);
        //intake2.setPower(0);
      //} else {
        //intake1.setPower(1);
        //intake2.setPower(1);
        //speedyleft.setPower(0);
        //speedyright.setPower(0);
      //}
      //region brennan takeover
      //speedyleft.setPower(gamepad1.right_trigger / speedMultiplyer );
      //speedyright.setPower(gamepad1.right_trigger / speedMultiplyer );
      
      intake1.setPower(gamepad1.right_trigger);
      intake2.setPower(gamepad1.right_trigger);
      
      if (gamepad1.a) {
        servolaunch.setPower(1);
        servo1.setPower(1);
      
        
      } else if (gamepad1.b) {
        servolaunch.setPower(-1);
        servo1.setPower(-1);
      
        
      } else if (gamepad2.a) {
        servolaunch.setPower(1);
        servo1.setPower(1);
      
        
      } else if (gamepad2.b) {
        servolaunch.setPower(-1);
        servo1.setPower(-1);
      
        
      } else {
        servolaunch.setPower(0);
        servo1.setPower(0);
      }
      
      //endregion
      
      if (gamepad1.dpadDownWasPressed()) {
        movementSpeed = .50;
      } else if (gamepad1.dpadUpWasPressed()) {
        movementSpeed = 1;
      }
      
      if (gamepad2.dpadDownWasPressed()) {
        speedMultiplyer = .50;
      } else if (gamepad2.dpadUpWasPressed()) {
        speedMultiplyer = .65;
      }
       /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
            odo.update();
            
            
             /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;
            
             /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            double heading = pos.getHeading(AngleUnit.DEGREES);
            telemetry.addData("Position", data);
            double longheading = Math.abs(pos.getHeading(AngleUnit.DEGREES) - 20);
            double shortheading = Math.abs(pos.getHeading(AngleUnit.DEGREES) - 45);
            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);
            
            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
if (longheading < 10) {  // Changed from > to 
    telemetry.addData("Longshot", "ready");
} 
if (shortheading < 10) {  // Changed from > to 
    telemetry.addData("Shortshot", "ready");
}
            
      // Show the elapsed game time and wheel power.
      telemetry.addData("Status", "Run Time: " + runtime);
      telemetry.addData("Front left/Right", JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(frontRightPower, 4, 2));
      telemetry.addData("Back  left/Right", JavaUtil.formatNumber(backLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(backRightPower, 4, 2));
      telemetry.update();
    }
  }
}
