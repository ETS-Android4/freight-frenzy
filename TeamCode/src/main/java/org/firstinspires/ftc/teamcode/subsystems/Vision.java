package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.BlueWarehouseTeamElementPipeline;
import org.firstinspires.ftc.teamcode.vision.RedCarouselTeamElementPipeline;
import org.firstinspires.ftc.teamcode.vision.BlueCarouselTeamElementPipeline;
import org.firstinspires.ftc.teamcode.vision.RedWarehouseTeamElementPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Vision implements Subsystem{
    public OpenCvCamera camera;
    private RedCarouselTeamElementPipeline elementPipelineRedCarousel = new RedCarouselTeamElementPipeline();
    private BlueCarouselTeamElementPipeline elementPipelineBlueCarousel = new BlueCarouselTeamElementPipeline();
    private BlueWarehouseTeamElementPipeline elementPipelineBlueWarehouse = new BlueWarehouseTeamElementPipeline();
    private RedWarehouseTeamElementPipeline elementPipelineRedWarehouse = new RedWarehouseTeamElementPipeline();

    public enum robotLocation {
        BLUE_CAROUSEL,
        RED_CAROUSEL,
        BLUE_WAREHOUSE,
        RED_WAREHOUSE
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
                        camera.setPipeline(elementPipelineRedCarousel);
                        break;
                    case BLUE_CAROUSEL:
                        camera.setPipeline(elementPipelineBlueCarousel);
                        break;
                    case BLUE_WAREHOUSE:
                        camera.setPipeline(elementPipelineBlueWarehouse);
                        break;
                    case RED_WAREHOUSE:
                        camera.setPipeline(elementPipelineRedWarehouse);
                        break;
                }
            }
            public void onError(int thing){

            }
        });
    }

    public void disable() {camera.stopStreaming();}

    public RedCarouselTeamElementPipeline getElementPipelineRedCarousel() { return elementPipelineRedCarousel;}

    public BlueCarouselTeamElementPipeline getElementPipelineBlueCarousel() { return elementPipelineBlueCarousel;}

    public BlueWarehouseTeamElementPipeline getElementPipelineBlueWarehouse() { return elementPipelineBlueWarehouse;}

    public RedWarehouseTeamElementPipeline getElementPipelineRedWarehouse() { return elementPipelineRedWarehouse;}



    @Override
    public void update(){}

    public void setRobotLocation (robotLocation starterLocation){
        this.startLocation = starterLocation;

    }

}
