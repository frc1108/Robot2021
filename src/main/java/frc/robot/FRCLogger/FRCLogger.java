package frc.robot.FRCLogger;

public class FRCLogger extends FileManager {
    public Csv csv;
  
    public FRCLogger(String file, String[] rows) {
      super(file);
  
      csv = new Csv(file, rows);
    }
  }