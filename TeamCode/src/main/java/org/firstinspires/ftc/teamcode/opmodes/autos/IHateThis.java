package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Config
@Autonomous
@Disabled
public class IHateThis extends RobotAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        vision.setRobotLocation(Vision.robotLocation.RED_CAROUSEL);
        vision.enable();



        while (!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Element Location: ", vision.getElementPipelineRedCarousel().getLocation());
            telemetry.update();
        }

        waitForStart();


        updateThread.start();

        //lift.setLiftPower(-1.0);

        lift.setExtensionState(Lift.ExtensionState.OUT);

        sleep(5000);

}
}