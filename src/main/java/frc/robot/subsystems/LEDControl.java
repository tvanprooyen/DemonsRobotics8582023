package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDControl extends SubsystemBase {
    private final PWM FRLED,BRLED,FLLED,BLLED;
    private AddressableLED m_Led;
    private AddressableLEDBuffer m_LedBuffer;
    private double zeroPos = 0;
    private int ledIndex = 0;
    private Boolean YellowToggle = true;
    private int FadeConstant = 0;
    private int FadeAdder = 5;

    public LEDAnimation selectedAnimation;

    public enum LEDAnimation {
        NONE(0),
        FADE(1),
        YELLOW(2),
        REDBLUE(3),
        CHASE(4),
        BLUE(5),
        PURPLE(6);


        private int selectedAnimation;

        LEDAnimation(int selectedAnimation) {
            this.selectedAnimation = selectedAnimation;
        }
    }

    public LEDControl() {
        FRLED = new PWM(0);
        BRLED = new PWM(1);
        FLLED = new PWM(8);
        BLLED = new PWM(9);

        this.selectedAnimation = LEDAnimation.NONE;
    }

    public void selectAnimation(LEDAnimation selectedAnimation) {
        this.selectedAnimation = selectedAnimation;
    }

    public LEDAnimation getLEDAnimation(){
        return this.selectedAnimation;
    }

    @Override
    public void periodic() {

        /* 
         NONE(0),
        FADE(1),
        YELLOW(2),
        REDBLUE(3),
        CHASE(4);
        BLUE(5)
         */

        switch (getLEDAnimation().selectedAnimation) {
            case 0: DisabledAnimation(); break;
            case 1: Fade(); break;
            case 2: Yellow(); break;
            case 3: FlashRedAndBlue(); break;
            case 4: ChaseLed(); break;
            case 5: Blue(); break;
            case 6: Purple(); break;
            default: Purple(); break;
        }

        m_Led.setData(m_LedBuffer);
    }

    
    public void DisabledAnimation() {
        DriverStation.Alliance color;
        color = DriverStation.getAlliance();
        if(color == DriverStation.Alliance.Blue){
        allColor(0,0,255);
        } else if(color == DriverStation.Alliance.Red) {
        allColor(255,0,0);
        } else {
        FlashRedAndBlue();
        }
    }

    private void Yellow(){
        allColor(255, 253, 153);
    }

    private void Blue(){
        allColor(0,0,255);
    }

    private void Purple(){
        allColor(255, 0, 200);
    }

  private void Fade(){
    for( int i = 0; i < m_LedBuffer.getLength(); i++){
      if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
        m_LedBuffer.setRGB(i,0,0,FadeConstant);
      } else {
        m_LedBuffer.setRGB(i,FadeConstant,0,0);
      }
    }

    if( FadeConstant + FadeAdder >= 256 || FadeConstant + FadeAdder <= 0 ){
      FadeAdder = -FadeAdder;
    } else {
      FadeConstant += FadeAdder;
    }
  
  }

  private void FlashRedAndBlue(){
    //reusing ledIndex number instead of timer because easier
  
    if( ledIndex < 40){
      allColor(255,0,0);
    } else if( ledIndex < 80){
      allColor(0,0,255);
    }
    ledIndex++;
    if(ledIndex >= 80) ledIndex = 0;
  }
  private void ChaseWhileHeld(int r,int b,int g){
    if( ledIndex + 5 < m_LedBuffer.getLength()){
      for( int i = 0; i < 4; i++){
        m_LedBuffer.setRGB(ledIndex+i,r, g, b);
      }
      ledIndex+=4;
    }
  }

  private void allColor(int r, int g, int b){
    for (int i = 0; i < m_LedBuffer.getLength(); i++) {
      m_LedBuffer.setRGB(i, r, g, b);
    }
  }

  private void ChaseLed(){
    zeroPos += 0.01;
    for( int i = 0; i < m_LedBuffer.getLength(); i++){
      double pctDownStrip = (double) i/ m_LedBuffer.getLength();
      double numCyclesOnStrip = (double) m_LedBuffer.getLength() / (double)30 / 2.0;
      double colorBump = Math.sin(2 * Math.PI * numCyclesOnStrip *  (pctDownStrip - zeroPos)) * 0.5 + 0.5;

      colorBump *= colorBump;
      colorBump *= m_LedBuffer.getLength();

      m_LedBuffer.setRGB(i, 84, ((int)colorBump - 0), ((int)colorBump - 153));
    }
  }
}
