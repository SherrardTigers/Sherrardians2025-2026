package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@TeleOp(name = "31893 competition (Voltage Comp)")
public class voltagecontrol extends LinearOpMode {

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
  private static final double NOMINAL_VOLTAGE = 13.5; // Target voltage
  private static final double BASE_LAUNCHER_POWER = 0.62; // Base power at 13.5V

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
    speedyleft.setDirection(DcMotor.Direction.REVERSE);
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

    telemetry.addData("Status", "Initialized");
    telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
    telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
    telemetry.addData("Device Version Number:", odo.getDeviceVersion());
    telemetry.addData("Heading Scalar", odo.getYawScalar());
    telemetry.update();
    
    waitForStart();
    runtime.reset();
    
    while (opModeIsActive()) {
      // Drive controls
      axial = -gamepad1.left_stick_y;
      lateral = gamepad1.left_stick_x;
      yaw = gamepad1.right_stick_x;
      
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
      // Get current battery voltage
      double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
      
      // Calculate voltage-compensated power based on trigger input
      double voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;
      double rawPower = gamepad2.right_trigger * speedMultiplyer;
      double compensatedPower = rawPower * voltageCompensation;
      
      // Clamp to safe range
      compensatedPower = Math.min(1.0, Math.max(0.0, compensatedPower));
      
      // Apply to launchers - trigger controls power directly
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
      
      // Speed adjustment
      if (gamepad1.dpad_down) {
        movementSpeed = .50;
      } else if (gamepad1.dpad_up) {
        movementSpeed = 1;
      }
      
      if (gamepad2.dpad_down) {
        speedMultiplyer = .46;
      } else if (gamepad2.dpad_up) {
        speedMultiplyer = .57;
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
      
      // VOLTAGE TELEMETRY - Shows compensation in action
      telemetry.addData("Battery Voltage", "%.2f V", currentVoltage);
      telemetry.addData("Trigger Input", "%.2f", gamepad2.right_trigger);
      telemetry.addData("Launcher Power (Raw)", "%.2f", rawPower);
      telemetry.addData("Launcher Power (Compensated)", "%.2f", compensatedPower);
      telemetry.addData("Voltage Compensation", "%.2f%%", voltageCompensation * 100);
      
      telemetry.addData("Status", "Run Time: " + runtime);
      telemetry.addData("Front left/Right", JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(frontRightPower, 4, 2));
      telemetry.addData("Back  left/Right", JavaUtil.formatNumber(backLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(backRightPower, 4, 2));
      telemetry.update();
    }
  }
}