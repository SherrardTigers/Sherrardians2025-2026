package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mecanum Power with Odometry", group="Linear Opmode")
public class OdometryWheels extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor intake = null;
    private DcMotor spinner = null;

    // Servo
    private Servo servo1 = null;
    private double servo1Position = 0.5;

    // Odometry encoders
    private DcMotor leftEncoder = null;
    private DcMotor rightEncoder = null;
    private DcMotor centerEncoder = null;

    // Odometry tracking variables
    private int prevLeft = 0, prevRight = 0, prevCenter = 0;
    private double x = 0.0, y = 0.0, heading = 0.0;

    // Constants
    private static final double TICKS_PER_REV = 8192.0;
    private static final double WHEEL_DIAMETER = 2.0; // inches
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private static final double TRACK_WIDTH = 14.0; // inches between left/right wheels

    @Override
    public void runOpMode() {

        // Initialize drive hardware
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        intake = hardwareMap.get(DcMotor.class, "intake");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        servo1 = hardwareMap.get(Servo.class, "servo1");

        // Initialize odometry encoders
        leftEncoder = hardwareMap.get(DcMotor.class, "left_encoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "right_encoder");
        centerEncoder = hardwareMap.get(DcMotor.class, "center_encoder");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.FORWARD);
        leftEncoder.setDirection(DcMotor.Direction.REVERSE); // adjust if needed

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // updated from FLOAT

        // Set initial servo position
        servo1.setPosition(servo1Position);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Mecanum drive
            double drive = Math.pow(-gamepad1.left_stick_y, 3);
            double strafe = Math.pow(gamepad1.left_stick_x, 3);
            double turn = Math.pow(gamepad1.right_stick_x, 3);

            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                                  Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Intake control
            if (gamepad2.right_bumper) {
                intake.setPower(1.0);
            } else if (gamepad2.left_bumper) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }

            // Spinner control
            if (gamepad2.a) {
                spinner.setPower(0.45);
            } else if (gamepad2.b) {
                spinner.setPower(-0.45);
            } else {
                spinner.setPower(0.0);
            }

            // Servo control
            if (gamepad2.dpad_left) {
                servo1Position = 0.27;
            } else if (gamepad2.dpad_up) {
                servo1Position = 0.5;
            } else if (gamepad2.dpad_right) {
                servo1Position = 0.6;
            }
            servo1.setPosition(servo1Position);

            // Odometry update
            int leftTicks = leftEncoder.getCurrentPosition();
            int rightTicks = rightEncoder.getCurrentPosition();
            int centerTicks = centerEncoder.getCurrentPosition();

            int deltaLeft = leftTicks - prevLeft;
            int deltaRight = rightTicks - prevRight;
            int deltaCenter = centerTicks - prevCenter;

            double leftInches = (deltaLeft / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
            double rightInches = (deltaRight / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
            double centerInches = (deltaCenter / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;

            double deltaHeading = (rightInches - leftInches) / TRACK_WIDTH;
            heading += deltaHeading;

            double forward = (leftInches + rightInches) / 2.0;
            double strafeInches = centerInches;

            x += forward * Math.cos(heading) - strafeInches * Math.sin(heading);
            y += forward * Math.sin(heading) + strafeInches * Math.cos(heading);

            prevLeft = leftTicks;
            prevRight = rightTicks;
            prevCenter = centerTicks;

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Front Left Power", "%.2f", frontLeftPower);
            telemetry.addData("Front Right Power", "%.2f", frontRightPower);
            telemetry.addData("Back Left Power", "%.2f", backLeftPower);
            telemetry.addData("Back Right Power", "%.2f", backRightPower);
            telemetry.addData("Intake Power", "%.2f", intake.getPower());
            telemetry.addData("Spinner Power", "%.2f", spinner.getPower());
            telemetry.addData("Servo Position", "%.2f", servo1Position);
            telemetry.addData("Odometry X", "%.2f", x);
            telemetry.addData("Odometry Y", "%.2f", y);
            telemetry.addData("Heading (rad)", "%.2f", heading);
            telemetry.update();
        }
    }
}