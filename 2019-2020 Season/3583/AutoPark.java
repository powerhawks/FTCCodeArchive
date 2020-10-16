package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park", group = "B Autos")
public class AutoPark extends AutoSkeletonSkystone {
    boolean tape = true;
    @Override
    public void runOpMode() {
        initRobot();
        while(!opModeIsActive()){
            if(gamepad1.x){
                tape = true;
            }else if(gamepad1.y){
                tape = false;
            }
            if(red){
                telemetry.addLine("Parking Mode: Tape Measure");
            }else{
                telemetry.addLine("Parking Mode: Movement");
            }
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){
            if(tape){
                tapeMeasure.setPower(1);
                sleep(500);
            }else {
                moveToPosition(10, .5);
            }
            requestOpModeStop();
        }
    }
}
