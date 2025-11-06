package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class DecodeChassis {

    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftBack;
    private final DcMotor rightBack;

    private static final double MAX_DRIVE_POWER = 1.0;
    private static final double LOW_SPEED_LIMIT = 0.30;

    public DecodeChassis(DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb) {
        leftFront = lf;
        rightFront = rf;
        leftBack = lb;
        rightBack = rb;

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double leftStickY, double leftStickX, double rightStickX, boolean lowSpeed) {
        double speedLimit = lowSpeed ? LOW_SPEED_LIMIT : MAX_DRIVE_POWER;

        // Mecanum drive calculation
        double fl = leftStickY + leftStickX + rightStickX;
        double fr = leftStickY - leftStickX - rightStickX;
        double bl = leftStickY - leftStickX + rightStickX;
        double br = leftStickY + leftStickX - rightStickX;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        fl = Range.clip(fl * speedLimit, -1.0, 1.0);
        fr = Range.clip(fr * speedLimit, -1.0, 1.0);
        bl = Range.clip(bl * speedLimit, -1.0, 1.0);
        br = Range.clip(br * speedLimit, -1.0, 1.0);

        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftBack.setPower(bl);
        rightBack.setPower(br);
    }
}



