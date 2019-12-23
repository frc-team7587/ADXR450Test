
package frc.team7587.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.team7587.robot.commands.AutoRotate;

public class OI {

  // Input devices
  public static final int LEFT_MOTOR = 0;
  public static final int RIGHT_MOTOR = 1;
  
  private final Joystick logiJoy = new Joystick(0);
  // private final Joystick gamePad = new Joystick(1); // gamepad

  private final Button btnRotate;
  private final Button btnRotate2;

  public OI() {
    
    // btnRotate = new JoystickButton(gamePad, 3);

    btnRotate = new JoystickButton(logiJoy, 2);
    btnRotate2 = new JoystickButton(logiJoy, 3);
    
    btnRotate.whenPressed(new AutoRotate(90f));
    btnRotate2.whenPressed(new AutoRotate(-90f));
  }

  public Joystick getLogiJoy() {
    return logiJoy;
  }

}
