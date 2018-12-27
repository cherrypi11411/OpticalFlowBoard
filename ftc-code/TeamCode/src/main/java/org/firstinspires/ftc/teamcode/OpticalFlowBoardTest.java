package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpticalFlowBoard;
import org.firstinspires.ftc.teamcode.DbgLog;

/**
 * Created by Cherry Pi on 12/27/2018.
 */
@TeleOp(name = "OpticalFlowBoardTest", group = "Testing")
public class OpticalFlowBoardTest extends LinearOpMode {
    OpticalFlowBoard opticalFlowBoard;

    @Override
    public void runOpMode() throws InterruptedException {
        DbgLog.msg("Starting opMode; retrieving the OpticalFlowBoard class");
        opticalFlowBoard = hardwareMap.get(OpticalFlowBoard.class, "opticalFlowBoard");
        DbgLog.msg("Fetching product id");
        byte productID = opticalFlowBoard.getProductIDRaw();
        byte cpProductID = opticalFlowBoard.getCPProductIDRaw();
        telemetry.addData("Product ID", productID);
        telemetry.addData("Cherry Pi Board Product ID", cpProductID);
        DbgLog.msg("productId: %02x cpProductId: %02x", productID, cpProductID);
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            opticalFlowBoard.setLedStatus(false, false, false, false);
            sleep(500);
            opticalFlowBoard.setLedStatus(false, true, false, false);
            sleep(500);
            OpticalFlowBoard.Absolute absolute = opticalFlowBoard.readAbsolute();
            if (absolute != null) {
                telemetry.addData("x", absolute.getX());
                telemetry.addData("y", absolute.getY());
                telemetry.addData("timestamp", absolute.getTimestamp());
            } else {
                telemetry.addData("absolute not read", "");
            }
            telemetry.update();
        }
    }
}
