package frc.robot.FRCLogger;

public class Csv extends FileManager {
  private String CompressedRows = "";
  private Boolean InitalWrite = false;

  public Csv(String filename, String[] rows) {
    super(filename);

    for (int i = 0; i != rows.length; i++) {
      String row = rows[i];

      // this is what the expected output is " row one,"
      row = " " + row + ",";

      // adds the currently formated row to compressed rows
      CompressedRows += row;
    }
  }

  public void LogWithTime(Object data) {
    if (!this.InitalWrite && this.CompressedRows != null) {
      this.Write("Time, " + this.CompressedRows + "\n");
      this.InitalWrite = true;
    }

    this.Write(java.time.LocalTime.now().toString() + "," + data + "\n");
  }
}