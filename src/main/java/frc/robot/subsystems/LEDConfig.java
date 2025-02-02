package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.LED.Blinker;
import frc.robot.LED.ParadoxAnim2;
public class LEDConfig {
    private final LEDSubsystem m_string1 = new LEDSubsystem(0, 45);
    private final LEDSubsystem m_string2 = new LEDSubsystem(45, 46, true);
    private final LEDSubsystem m_string3 = new LEDSubsystem(45 + 46, 30);

    private final Robot m_robot;

    // private Command m_currentCommand = null;

    @SuppressWarnings("unused")
    private ParallelCommandGroup m_paradox = new ParallelCommandGroup(new ParadoxAnim2(m_string1),
            new ParadoxAnim2(m_string2), new ParadoxAnim2(m_string3),
    // private ParallelCommandGroup m_cameraFailure = new ParallelCommandGroup(new Blinker(m_string1, 0.5, Color.kBlue),
            new Blinker(m_string2, 0.5, Color.kBlue), new Blinker(m_string3, 0.5, Color.kBlue));
    private ParallelCommandGroup m_coral = new ParallelCommandGroup(new Blinker(m_string1, 0, Color.kPurple),
            new Blinker(m_string2, 0, Color.kPurple), new Blinker(m_string3, 0, Color.kPurple));
    private ParallelCommandGroup m_coralNote = new ParallelCommandGroup(new Blinker(m_string1, 0.2, Color.kPurple),
            new Blinker(m_string2, 0.2, Color.kPurple), new Blinker(m_string3, 0.2, Color.kPurple));
    private ParallelCommandGroup m_algae = new ParallelCommandGroup(new Blinker(m_string1, 0, Color.kGreen),
            new Blinker(m_string2, 0, Color.kGreen), new Blinker(m_string3, 0, Color.kGreen));
    private ParallelCommandGroup m_algaeNote = new ParallelCommandGroup(new Blinker(m_string1, 0.2, Color.kGreen),
            new Blinker(m_string2, 0.2, Color.kGreen), new Blinker(m_string3, 0.2, Color.kGreen));
    private ParallelCommandGroup m_cameraError;

    public LEDConfig(Robot robot/*, ApriltagsCamera cameraFrontBack, ApriltagsCamera cameraLeftRight*/) {
        m_robot = robot;
        // m_cameraFrontBack = cameraFrontBack;
        // m_cameraLeftRight = cameraLeftRight;

        // m_cameraError = new ParallelCommandGroup(new DisplayCameraError(m_string1, m_cameraFrontBack), new DisplayCameraError(m_string2, m_cameraFrontBack), new DisplayCameraError(m_string3, m_cameraFrontBack));
    }

    public void periodic() {
        // Command command = null;

       
}
}
