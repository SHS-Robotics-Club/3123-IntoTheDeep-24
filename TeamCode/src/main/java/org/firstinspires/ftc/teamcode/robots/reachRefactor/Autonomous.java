package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

public class Autonomous {
    public VisionProvider visionProvider;
    private Robot robot;

    public Autonomous(Robot robot) {
        this.robot = robot;
    }

    private StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> {})
                .stateEndAction(() -> {})
                .stage(stage);
    }

    public void createVisionProvider(int visionProviderIndex) {
        try {
            visionProvider = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance();
        } catch(IllegalAccessException | InstantiationException e) {
//            throw new RuntimeException("Error while instantiating vision provider");
        }
    }

    // Autonomous articulations
    private Stage autonomousRedStage = new Stage();
    public StateMachine autonomousRed = getStateMachine(autonomousRedStage)
            .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                    () -> true,
                    () -> true,
                    () -> true
            )
            .build();

    private Stage autonomousBlueStage = new Stage();
    public StateMachine autonomousBlue = getStateMachine(autonomousBlueStage)
            .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                    () -> true,
                    () -> true,
                    () -> true
            )
            .build();
}
