package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class DecodeIntake {

    private final DcMotor intakeMotor;
    private static final double LOW_SPEED_LIMIT = 0.30;

    public DecodeIntake(DcMotor intake) {
        intakeMotor = intake;
    }

    public void run(boolean forwardButton, boolean backwardButton, double triggerValue, boolean lowSpeed) {
        double power = 0.0;

        if (backwardButton) {
            power = -1.0; // full backward
        } else if (forwardButton) {
            power = Range.clip(triggerValue, 0.0, 1.0); // forward power based on trigger
        }

        if (lowSpeed) {
            power = Range.clip(power, -LOW_SPEED_LIMIT, LOW_SPEED_LIMIT);
        }

        intakeMotor.setPower(power);
    }
}


