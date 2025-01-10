package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Constants;
import frc.robot.Robot;

public class LEDConfig {
    private final LEDSubsystem m_string1 = new LEDSubsystem(0, 45);
    private final LEDSubsystem m_string2 = new LEDSubsystem(45, 46, true);
    private final LEDSubsystem m_string3 = new LEDSubsystem(45 + 46, 30);

    private final Robot m_robot;

    private Command m_currentCommand = null;

    @SuppressWarnings("unused")

    public LEDConfig(Robot robot) {
        m_robot = robot;
    }

    public void periodic() {
        Command command = null;

        if (command != m_currentCommand) {
            m_currentCommand = command;
            m_currentCommand.schedule();
        }
    }
}
