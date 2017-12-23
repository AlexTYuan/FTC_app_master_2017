package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
/**
 * Created by alexyuan on 11/17/17.
 */

@TeleOp(name = "L3G4200D", group = "test")
@Disabled
public class L3G4200DTest extends LinearOpMode {

    private L3G4200D gyroSensor;

    public void runOpMode() throws InterruptedException {
        gyroSensor = hardwareMap.get(L3G4200D.class, "gyro");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Manufacturer ID", gyroSensor.getManufacturerIDRaw());
            telemetry.addData("X Angle", gyroSensor.getX());
            telemetry.addData("Y Angle", gyroSensor.getY());
            telemetry.addData("Z Angle", gyroSensor.getZ());
            telemetry.update();
            idle();
        }
    }
}
