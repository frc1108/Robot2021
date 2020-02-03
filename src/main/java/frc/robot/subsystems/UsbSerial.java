
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.lib.CR16;

public class UsbSerial extends SubsystemBase {

    private final SerialPort ArduinoSerial = new SerialPort(115200, SerialPort.Port.kUSB);
    byte[] buffer = new byte[7];
    int crc;

    public void getArduino(){

        /*
        // test code for arduino crc. 
        buffer[0] = 0x21;
        buffer[1] = 0x04;//Function Code; in this case 0x04 is the read command
        buffer[2] = 0;
        buffer[3] = 20;//Starting address of the register
        buffer[4] = 0;
        buffer[5] = 10;//Number of registers to read
        
        // ArduinoSerial.readString();
        crc = CR16.getCRC16(buffer);
        //System.out.printf("%x \n", crc); 
        */
    }
}