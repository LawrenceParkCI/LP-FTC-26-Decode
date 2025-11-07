package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FinalDecode", group = "Linear Opmode")
public class FinalDecode extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Shooter, Intake, Carousel
    private DcMotorEx shooter;
    private DcMotor intake;
    private DcMotor carousel;

    // Servo
    private Servo pusher;

    // Shooter variables
    private double currentRPM = 0;
    private double targetRPM = 0;
    private boolean targetMet = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        carousel = hardwareMap.get(DcMotor.class, "carousel");

        pusher = hardwareMap.get(Servo.class, "pusher");

        // Motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servo init
        pusher.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            // --- DRIVE CONTROLS ---
            double drive = -gamepad1.left_stick_y; // forward/back
            double strafe = gamepad1.left_stick_x; // left/right
            double spin = gamepad1.right_stick_x; // rotation

            boolean lowSpeed = gamepad1.right_trigger > 0;

            double maxPower = lowSpeed ? 0.3 : 1.0;

            // Calculate powers
            double lfPower = clip(drive + strafe + spin, maxPower);
            double rfPower = clip(drive - strafe - spin, maxPower);
            double lbPower = clip(drive - strafe + spin, maxPower);
            double rbPower = clip(drive + strafe - spin, maxPower);

            leftFront.setPower(lfPower);
            rightFront.setPower(rfPower);
            leftBack.setPower(lbPower);
            rightBack.setPower(rbPower);

            // --- INTAKE CONTROLS ---
            double intakePower = 0;
            if (gamepad1.left_bumper) intakePower = -1; // backwards
            else if (gamepad1.left_trigger > 0) intakePower = gamepad1.left_trigger; // forward

            if (gamepad1.a) intakePower *= 0.3; // low-speed mode

            intake.setPower(intakePower);

            // --- SHOOTER CONTROLS ---
            if (gamepad2.x) targetRPM = 1000 * 0.9;
            else if (gamepad2.a) targetRPM = 2500 * 0.9;
            else if (gamepad2.b) targetRPM = 5000 * 0.9;
            else if (gamepad2.y) targetRPM = 0;

            shooter.setVelocity(targetRPM); // set velocity in ticks/sec
            currentRPM = shooter.getVelocity();
            targetMet = currentRPM >= targetRPM;

            // --- PUSHER ---
            if (gamepad2.right_bumper) {
                pusher.setPosition(0.266); // 80 deg / 300 deg range
                sleep(200); // short delay
                pusher.setPosition(0); // back to 0
            }

            // --- CAROUSEL ---
            if (gamepad2.dpad_up) rotateCarousel(60);
            else if (gamepad2.dpad_down) rotateCarousel(-60);
            else if (gamepad2.dpad_right) rotateCarousel(120);
            else if (gamepad2.dpad_left) rotateCarousel(-120);

            // --- TELEMETRY ---
            telemetry.addData("Shooter RPM", currentRPM);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Target Met", targetMet);
            telemetry.update();
        }
    }

    private double clip(double value, double max) {
        if (value > max) return max;
        if (value < -max) return -max;
        return value;
    }

    private void rotateCarousel(int degrees) {
        int targetPos = carousel.getCurrentPosition() + (int)(degrees * 28); // 28 ticks per deg approx
        carousel.setTargetPosition(targetPos);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setPower(0.5);
        while (opModeIsActive() && carousel.isBusy()) {
            // wait until done
        }
        carousel.setPower(0);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

