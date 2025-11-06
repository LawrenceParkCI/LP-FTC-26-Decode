package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class DecodeShooter {

    private final DcMotorEx shooter;
    private final DcMotorEx carousel;
    private final Servo pusher;

    private double currentRPM = 0.0;
    private double targetRPM = 0.0;
    private boolean targetMet = false;

    private int carouselTargetTicks = 0;

    private static final double TICKS_PER_REV = 28; // GoBilda 5202 motor
    private static final double CAROUSEL_GEAR_RATIO = 99.5;
    private static final double TICKS_PER_DEGREE_CAROUSEL = (TICKS_PER_REV * CAROUSEL_GEAR_RATIO) / 360.0;
    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_MOVE_DEGREES = 80.0;
    private static final double SERVO_MOVE_POSITION = SERVO_CENTER + (SERVO_MOVE_DEGREES / 300.0);

    public DecodeShooter(DcMotorEx shooterMotor, DcMotorEx carouselMotor, Servo pusherServo) {
        shooter = shooterMotor;
        carousel = carouselMotor;
        pusher = pusherServo;

        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setTargetPosition(0);
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setPower(0);

        pusher.setPosition(SERVO_CENTER);
    }

    public void setShooterRPM(double rpm) {
        targetRPM = rpm * 0.9; // per spec
        double motorFreeRPM = 6000.0;
        double power = Range.clip(targetRPM / motorFreeRPM, 0.0, 1.0);
        shooter.setPower(power);
    }

    public void updateCurrentRPM() {
        try {
            double ticksPerSecond = shooter.getVelocity();
            currentRPM = (ticksPerSecond / TICKS_PER_REV) * 60.0;
            targetMet = currentRPM >= targetRPM;
        } catch (Exception e) {
            currentRPM = 0;
            targetMet = targetRPM == 0;
        }
    }

    public double getCurrentRPM() { return currentRPM; }
    public double getTargetRPM() { return targetRPM; }
    public boolean isTargetMet() { return targetMet; }

    public void pushServo() {
        pusher.setPosition(SERVO_MOVE_POSITION);
        try { Thread.sleep(180); } catch (InterruptedException ignored) {}
        pusher.setPosition(SERVO_CENTER);
    }

    public void rotateCarousel(double degrees) {
        carouselTargetTicks += (int)Math.round(degrees * TICKS_PER_DEGREE_CAROUSEL);
        carouselTargetTicks = (carouselTargetTicks % 360 + 360) % 360; // keep in 0-360
        carousel.setTargetPosition(carouselTargetTicks);
        carousel.setPower(0.5);
        while (carousel.isBusy()) {}
        carousel.setPower(0);
    }
}


