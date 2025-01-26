package frc.robot.robotControl;

import java.util.Timer;
import java.util.TimerTask;

import frc.robot.robotControl.Network.NetworkReceiver;

public class RobotControl implements NetworkReceiver {
    private Network m_network = new Network();
    private Timer m_watchdogTimer = new Timer();
    private boolean m_connected = false;
    private double m_xPos = 0;
    private double m_yPos = 0;
    private boolean m_newPos = true;
    private Object m_lock = new Object();
    private final int k_maxButtons=30;

    private boolean[] m_buttons = new boolean[k_maxButtons];

    // double m_testX = 10*12;
    // double m_testY = 0;
    double m_angle = 90;

    public void start() {
        // m_gyro = gyro;
        m_network.listen(this, 5803);

        m_watchdogTimer.scheduleAtFixedRate(new TimerTask() {

			@Override
			public void run() {
                // Logger.log("RobotControl", 1, "connected=" + m_connected);
				if (m_connected) {
					Logger.log("RobotControl",-1, "Send position");

                    double xPos;
                    double yPos;
                    double angle;
                    boolean newPos;

                    synchronized (m_lock)
                    {
                        xPos = m_xPos;
                        yPos = m_yPos;
                        angle = m_angle;
                        newPos = m_newPos;

                        m_newPos = false;
                    }

                    if (newPos)
                    {
					    m_network.sendMessage(String.format("+%.2f %.2f %.2f\n", angle, xPos, yPos));
                    }
                    else
                    {
                        m_network.sendMessage("-\n");       // keep alive
                    }

					// if (m_lastMessage + k_timeout < System.currentTimeMillis()) {
					// 	Logger.log("ApriltagsCamera", 3, "Network timeout");
					// 	m_network.closeConnection();
					// }
				}
			}
		}, 200, 200);   // Send current position 5 times a second
    }

    public void setPosition(double x, double y, double angle) {
        synchronized (m_lock) {
            m_xPos = x * 12;
            m_yPos = y * 12;
            m_angle = angle;
            m_newPos = true;
        }
    }

    // TODO: We should expose Triggers for buttons. -Gavin

    public boolean checkButton(int button) {
        if ((button < 0) || (button >= k_maxButtons)) {
            Logger.log("RobotContainer", 3, String.format("checkButton: Invalid button: %d", button));
            return false;
        }

        synchronized (m_lock) {
            boolean ret = m_buttons[button];
            m_buttons[button] = false;
            return(ret);
        }
    }

    private void buttonPressed(String command) {
        int buttonNo;

        command = command.stripTrailing();

        // Logger.log("RobotControl", 1, String.format("buttonPressed: '%s'", command));
        
        try {
            buttonNo = Integer.parseInt(command.stripTrailing());

            // Logger.log("RobotControl", 1, String.format("buttonPressed: buttonNo=%d", buttonNo));

            if ((buttonNo < 0) || (buttonNo >= k_maxButtons)) {
                Logger.log("RobotControl", 3, String.format("Invalid button number: %d", buttonNo));
            }
            else {
                synchronized (m_lock) {
                    m_buttons[buttonNo] = true;
                }
            }
        } catch (NumberFormatException ex) {
            Logger.log("RobotControl", 3, String.format("Invalid button command: '%s'", command));
        }

    }

    @Override
    public void processData(String data) {
        Logger.log("RobotControl", -1, String.format("Data: %s", data));

        switch (data.charAt(0)) {
            case 'k':   // keep alive
                break;

            case 'B':   // button
                buttonPressed(data.substring(2));
                break;

            default:
                Logger.log("RobotControl", 3, String.format("Invalid command: %s", data));
                break;
        }
    }

    @Override
    public void connected() {
        Logger.log("RobotControl", 1, String.format("connected"));

        m_connected = true;
    }

    @Override
    public void disconnected() {
        Logger.log("RobotControl", 1, "disconnected");

        m_connected = false;
    }

}
