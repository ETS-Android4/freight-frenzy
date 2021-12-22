package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.RedCarouselTeamElementPipeline;
import org.firstinspires.ftc.teamcode.vision.BlueCarouselTeamElementPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Vision implements Subsystem{
    public OpenCvCamera camera;
    private RedCarouselTeamElementPipeline elementPipelineRed = new RedCarouselTeamElementPipeline();
    private BlueCarouselTeamElementPipeline elementPipelineBlue = new BlueCarouselTeamElementPipeline();

    public enum robotLocation {
        BLUE_CAROUSEL,
        RED_CAROUSEL
    }

    public robotLocation startLocation = robotLocation.BLUE_CAROUSEL;

    public Vision(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

    }



    public void enable(){
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                switch (startLocation){
                    case RED_CAROUSEL:
                        camera.setPipeline(elementPipelineRed);
                        break;
                    case BLUE_CAROUSEL:
                        camera.setPipeline(elementPipelineBlue);
                }
            }
            public void onError(int thing){

            }
        });
    }

    public void disable() {camera.stopStreaming();}

    public RedCarouselTeamElementPipeline getElementPipelineRed() { return elementPipelineRed;}

    public BlueCarouselTeamElementPipeline getElementPipelineBlue() { return elementPipelineBlue;}



    @Override
    public void update(){}

    public void setRobotLocation (robotLocation starterLocation){
        this.startLocation = starterLocation;

    }

}
