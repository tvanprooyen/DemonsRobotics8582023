package frc.robot.Util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("deprecation")
public class JoystickAxisAsButton extends Button {
  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public JoystickAxisAsButton(GenericHID joystick, int axisNumber) {
    super(() -> joystick.getRawAxis(axisNumber) > 0.02);
    requireNonNullParam(joystick, "joystick", "JoystickAxisAsButton");
  }
}
