import static org.junit.jupiter.api.Assertions.assertEquals;
import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.ElevatorSubsystem;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.sim.SparkFlexSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestElevatorSubsystem {
    private ElevatorSubsystem m_elevatorSubsystem;
    private SparkFlexSim m_simElevatorMotor;

    @BeforeEach
    public void setup() {
        assert(HAL.initialize(500, 0));
        m_elevatorSubsystem = new ElevatorSubsystem();
        m_simElevatorMotor = m_elevatorSubsystem.getSimMotor();  
    }

    @Test
    public void testPower() {        
        assertEquals(m_simElevatorMotor.getAppliedOutput(), 0.0);
        m_elevatorSubsystem.setPower(0.5);
        assertEquals(m_simElevatorMotor.getAppliedOutput(), 0.5);
        m_elevatorSubsystem.setPower(1.0);
        assertEquals(m_simElevatorMotor.getAppliedOutput(), 1.0);
    }

    @Test void testPosition() {
        assertEquals(m_simElevatorMotor.getSetpoint(), 0);
        m_elevatorSubsystem.setPosition(0.5);
        assertEquals(m_simElevatorMotor.getSetpoint(), 0.5);
        m_elevatorSubsystem.setPosition(1.0);
        assertEquals(m_simElevatorMotor.getSetpoint(), 1.0);
    }

    @Test void testPosition2() {
        m_elevatorSubsystem.setPosition(1.0);
        for (int i = 0; i < 100; i++) {
            m_elevatorSubsystem.simulationPeriodic();
        }
        assertEquals(m_simElevatorMotor.getPosition(), 1.0, 0.02);
    }

    @AfterEach
    public void tearDown() {
        m_elevatorSubsystem.close();
    }
}
