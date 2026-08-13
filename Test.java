package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Blue Launch Velocity", group="Auto")
public class Test extends LinearOpMode {
    
    // Motors - drive uses power, launcher uses velocity
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx speedyleft;
    private DcMotorEx intake2;
    private DcMotorEx speedyright;
    private DcMotorEx intake1;
    private CRServo servolaunch;
    private CRServo servo1;
    
    // Pinpoint odometry
    private GoBildaPinpointDriver odo;
    
    // Movement parameters
    private static final double MAX_SPEED = 0.8;
    private static final double MIN_SPEED = 0.15;
    private static final double SLOWDOWN_DISTANCE = 6.0; // inches - start slowing down
    private static final double POSITION_TOLERANCE = 0.3; // inches
    private static final double HEADING_TOLERANCE = 1.5; // degrees
    
    // Launcher velocity parameters - GoBILDA 6000 RPM motors
    private static final double LAUNCHER_VELOCITY = 30; // ticks per second - TUNE THIS
    private static final double TICKS_PER_REV = 28.0; // GoBILDA 6000 RPM (Yellow Jacket)
    
    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Velocity Control");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // YOUR AUTONOMOUS SEQUENCE HERE
            forward(-6);
            rotate(5);
            strafe(10);
            rotate(-25);
            fire();
            rotate(25);
            strafe(-10);
            forward(-12);
        }
    }
    
    private void initHardware() {
        // Initialize drive motors as regular DcMotor
        frontLeft = hardwareMap.get(DcMotor.class, "motor 1");
        frontRight = hardwareMap.get(DcMotor.class, "motor 3");
        backLeft = hardwareMap.get(DcMotor.class, "motor 2");
        backRight = hardwareMap.get(DcMotor.class, "motor 4");
        
        // Initialize launcher motors as DcMotorEx for velocity control
        speedyleft = hardwareMap.get(DcMotorEx.class, "speedy left");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake 2");
        speedyright = hardwareMap.get(DcMotorEx.class, "speedy right");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake 1");
        servolaunch = hardwareMap.get(CRServo.class, "servo launch");
        servo1 = hardwareMap.get(CRServo.class, "servo1");

        // Set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        speedyleft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Set launcher motors to use velocity control
        speedyleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedyright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedyleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        speedyright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Initialize Pinpoint odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                                 GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }
    
    /**
     * Move forward the specified number of inches
     */
    public void forward(double mmininches) {
        double inches = (mmininches * 25.4);
        odo.update();
        double startY = odo.getPosY(DistanceUnit.MM);
        double targetY = startY + inches;
        
        telemetry.addData("Forward", "%.1f inches", inches);
        telemetry.update();
        
        while (opModeIsActive()) {
            odo.update();
            double currentY = odo.getPosY(DistanceUnit.MM);
            double remaining = targetY - currentY;
            
            if (Math.abs(remaining) < POSITION_TOLERANCE) {
                break;
            }
            
            if ((inches > 0 && currentY > targetY) || (inches < 0 && currentY < targetY)) {
                break;
            }
            
            // Calculate speed with slowdown
            double speed = calculateSpeed(Math.abs(remaining));
            if (remaining < 0) speed = -speed;
            
            // Mecanum forward: all wheels same direction
            frontLeft.setPower(-speed / 2);
            frontRight.setPower(-speed / 2);
            backLeft.setPower(-speed / 2);
            backRight.setPower(-speed / 2);
            
            telemetry.addData("Current Y", "%.1f mm", currentY);
            telemetry.addData("Target Y", "%.1f mm", targetY);
            telemetry.addData("Speed", "%.2f", speed);
            telemetry.update();
        }
        
        stopMotors();
        sleep(100);
    }
    
    /**
     * Move backward the specified number of inches
     */
    public void backward(double inches) {
        forward(-inches);
    }
    
    /**
     * Strafe right (positive) or left (negative) the specified number of inches
     */
    public void strafe(double mmininch) {
        double inches = (mmininch * 25.4);
        odo.update();
        double startX = odo.getPosX(DistanceUnit.MM);
        double targetX = startX + inches;
        
        telemetry.addData("Strafe", "%.1f inches", inches);
        telemetry.update();
        
        while (opModeIsActive()) {
            odo.update();
            double currentX = odo.getPosX(DistanceUnit.MM);
            double remaining = targetX - currentX;
            
            if (Math.abs(remaining) < POSITION_TOLERANCE) {
                break;
            }
            
            if ((inches > 0 && currentX > targetX) || (inches < 0 && currentX < targetX)) {
                break;
            }
            
            // Calculate speed with slowdown
            double speed = calculateSpeed(Math.abs(remaining));
            if (remaining < 0) speed = -speed;
            
            // Mecanum strafe right: FL/BR forward, FR/BL backward
            frontLeft.setPower(speed / 2);
            frontRight.setPower(-speed / 2);
            backLeft.setPower(-speed / 2);
            backRight.setPower(speed / 2);
            
            telemetry.addData("Current X", "%.1f mm", currentX);
            telemetry.addData("Target X", "%.1f mm", targetX);
            telemetry.addData("Speed", "%.2f", speed);
            telemetry.update();
        }
        
        stopMotors();
        sleep(100);
    }
    
    /**
     * Rotate clockwise (positive) or counterclockwise (negative) in degrees
     */
    public void rotate(double degrees) {
        odo.update();
        double startHeading = odo.getHeading(AngleUnit.DEGREES);
        double targetHeading = startHeading - degrees;
        
        // Normalize to -180 to 180
        while (targetHeading > 180) targetHeading -= 360;
        while (targetHeading < -180) targetHeading += 360;
        
        telemetry.addData("Rotate", "%.1f degrees", degrees);
        telemetry.update();
        
        while (opModeIsActive()) {
            odo.update();
            double currentHeading = odo.getHeading(AngleUnit.DEGREES);
            double error = targetHeading - currentHeading;
            
            // Normalize error to -180 to 180
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
            
            if (Math.abs(error) < HEADING_TOLERANCE) {
                break;
            }
            
            // Calculate turn speed with slowdown (slowdown starts at 30 degrees)
            double speed = calculateSpeed(Math.abs(error) / 5.0); // Scale degrees to inches equivalent
            speed = Math.max(0.12, Math.min(0.5, speed)); // Limit turn speed range
            if (error < 0) speed = -speed;
            
            // Mecanum rotate: left side forward, right side backward
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(speed);
            backRight.setPower(-speed);
            
            telemetry.addData("Current Heading", "%.1f°", currentHeading);
            telemetry.addData("Target Heading", "%.1f°", targetHeading);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("Speed", "%.2f", speed);
            telemetry.update();
        }
        
        stopMotors();
        sleep(100);
    }
    
    /**
     * Calculate speed based on remaining distance with slowdown
     */
    private double calculateSpeed(double remainingDistance) {
        if (remainingDistance > SLOWDOWN_DISTANCE) {
            return MAX_SPEED;
        } else {
            // Linear slowdown from MAX_SPEED to MIN_SPEED
            double ratio = remainingDistance / SLOWDOWN_DISTANCE;
            return MIN_SPEED + (MAX_SPEED - MIN_SPEED) * ratio;
        }
    }
    
    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    
    private void fire() {
        // Use velocity control for consistent launcher speed
        speedyleft.setVelocity(LAUNCHER_VELOCITY);
        speedyright.setVelocity(LAUNCHER_VELOCITY);
        
        for (int count = 0; count < 0; count++) {
            sleep(1500);
            servo1.setPower(-1);
            sleep(1500);
            servolaunch.setPower(0);
            servo1.setPower(0);
        }
        
        for (int count2 = 0; count2 < 5; count2++) {
            sleep(1500);
            servolaunch.setPower(-1);
            servo1.setPower(-1);
            sleep(400);
            servolaunch.setPower(0);
            servo1.setPower(0);
        }
        
        sleep(200);
        speedyleft.setVelocity(0);
        speedyright.setVelocity(0);
    }
}