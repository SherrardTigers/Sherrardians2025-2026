package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "redpark", group = "Autonomous")
public class TimedAuto extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeft  = hardwareMap.get(DcMotor.class, "motor 1");
        frontRight = hardwareMap.get(DcMotor.class, "motor 3");
        backLeft   = hardwareMap.get(DcMotor.class, "motor 2");
        backRight  = hardwareMap.get(DcMotor.class, "motor 4");

        // Reverse left side motors (typical for mecanum/west coast mirrored builds)
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Ensure time-based (no encoders)
        setRunWithoutEncoder();

        // Optional but recommended so the bot doesn't coast after timed moves
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (!opModeIsActive()) return;

        // === SAMPLE SEQUENCE (adjust durations as you like) ===
        driveForTime(750, 0.5);    // drive forward for 1.5s
        strafeForTime(2000, -0.5);
        
        stopMotors();
    }

    // ---------- Time-based motion helpers ----------
    private void driveForTime(long millis, double power) {
        // Forward/back: all motors same sign
        power = clip(power);
        setRunWithoutEncoder();

        setMotorPowers(power, power, power, power);
        runForMillis(millis, "Driving", power, power, power, power);
        stopMotors();
    }

    private void strafeForTime(long millis, double power) {
        // Mecanum strafe: + - - +
        power = clip(power);
        setRunWithoutEncoder();

        double fl =  power;
        double fr = -power;
        double bl = -power;
        double br =  power;

        setMotorPowers(fl, fr, bl, br);
        runForMillis(millis, "Strafing", fl, fr, bl, br);
        stopMotors();
    }

    private void rotateForTime(long millis, double power) {
        // In-place rotate: left -, right +
        power = clip(power);
        setRunWithoutEncoder();

        double fl = -power;
        double fr =  power;
        double bl = -power;
        double br =  power;

        setMotorPowers(fl, fr, bl, br);
        runForMillis(millis, "Rotating", fl, fr, bl, br);
        stopMotors();
    }

    // ---------- Utilities ----------
    private void runForMillis(long millis, String phase,
                              double fl, double fr, double bl, double br) {
        ElapsedTime t = new ElapsedTime();
        t.reset();
        while (opModeIsActive() && t.milliseconds() < millis) {
            telemetry.addData("%s ms", phase, millis);
            telemetry.addData("Remaining", Math.max(0, millis - (long)t.milliseconds()));
            telemetry.addData("Powers", "FL: %.2f FR: %.2f BL: %.2f BR: %.2f", fl, fr, bl, br);
            telemetry.update();
            idle(); // be nice to the scheduler
        }
    }

    private void setRunWithoutEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    private double clip(double p) {
        return Math.max(-1.0, Math.min(1.0, p));
    }
}
