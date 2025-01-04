package frc.robot.subsystems.vision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Mt2VisionPose", mt2VisionPose);
    table.put("Mt1VisionPose", mt1VisionPose);
    table.put("TimestampSeconds", timestampSeconds);
    table.put("TagCount", tagCount);
    table.put("Latency", latency);
    table.put("TagSpan", tagSpan);
    table.put("AvgTagDist", avgTagDist);
    table.put("AvgTagArea", avgTagArea);
    table.put("ITX", iTX);
    table.put("ITY", iTY);
    table.put("ITA", iTA);
    table.put("IHB", iHB);
    table.put("ITV", iTV);
    table.put("ITHOR", iTHOR);
    table.put("ITVERT", iTVERT);
    table.put("IPIPELINELATENCY", iPIPELINELATENCY);
    table.put("ICAPTURELATENCY", iCAPTURELATENCY);
    table.put("ATX", aTX);
    table.put("ATY", aTY);
    table.put("ATA", aTA);
    table.put("AHB", aHB);
    table.put("ATV", aTV);
    table.put("ATHOR", aTHOR);
    table.put("ATVERT", aTVERT);
    table.put("APIPELINELATENCY", aPIPELINELATENCY);
    table.put("ACAPTURELATENCY", aCAPTURELATENCY);
  }

  @Override
  public void fromLog(LogTable table) {
    mt2VisionPose = table.get("Mt2VisionPose", mt2VisionPose);
    mt1VisionPose = table.get("Mt1VisionPose", mt1VisionPose);
    timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
    tagCount = table.get("TagCount", tagCount);
    latency = table.get("Latency", latency);
    tagSpan = table.get("TagSpan", tagSpan);
    avgTagDist = table.get("AvgTagDist", avgTagDist);
    avgTagArea = table.get("AvgTagArea", avgTagArea);
    iTX = table.get("ITX", iTX);
    iTY = table.get("ITY", iTY);
    iTA = table.get("ITA", iTA);
    iHB = table.get("IHB", iHB);
    iTV = table.get("ITV", iTV);
    iTHOR = table.get("ITHOR", iTHOR);
    iTVERT = table.get("ITVERT", iTVERT);
    iPIPELINELATENCY = table.get("IPIPELINELATENCY", iPIPELINELATENCY);
    iCAPTURELATENCY = table.get("ICAPTURELATENCY", iCAPTURELATENCY);
    aTX = table.get("ATX", aTX);
    aTY = table.get("ATY", aTY);
    aTA = table.get("ATA", aTA);
    aHB = table.get("AHB", aHB);
    aTV = table.get("ATV", aTV);
    aTHOR = table.get("ATHOR", aTHOR);
    aTVERT = table.get("ATVERT", aTVERT);
    aPIPELINELATENCY = table.get("APIPELINELATENCY", aPIPELINELATENCY);
    aCAPTURELATENCY = table.get("ACAPTURELATENCY", aCAPTURELATENCY);
  }

  public VisionIOInputsAutoLogged clone() {
    VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.mt2VisionPose = this.mt2VisionPose;
    copy.mt1VisionPose = this.mt1VisionPose;
    copy.timestampSeconds = this.timestampSeconds;
    copy.tagCount = this.tagCount;
    copy.latency = this.latency;
    copy.tagSpan = this.tagSpan;
    copy.avgTagDist = this.avgTagDist;
    copy.avgTagArea = this.avgTagArea;
    copy.iTX = this.iTX;
    copy.iTY = this.iTY;
    copy.iTA = this.iTA;
    copy.iHB = this.iHB;
    copy.iTV = this.iTV;
    copy.iTHOR = this.iTHOR;
    copy.iTVERT = this.iTVERT;
    copy.iPIPELINELATENCY = this.iPIPELINELATENCY;
    copy.iCAPTURELATENCY = this.iCAPTURELATENCY;
    copy.aTX = this.aTX;
    copy.aTY = this.aTY;
    copy.aTA = this.aTA;
    copy.aHB = this.aHB;
    copy.aTV = this.aTV;
    copy.aTHOR = this.aTHOR;
    copy.aTVERT = this.aTVERT;
    copy.aPIPELINELATENCY = this.aPIPELINELATENCY;
    copy.aCAPTURELATENCY = this.aCAPTURELATENCY;
    return copy;
  }
}
