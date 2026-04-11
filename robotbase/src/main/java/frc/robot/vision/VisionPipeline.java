package frc.robot.vision;

public enum VisionPipeline {
  MULTI_TAG_PIPELINE(0),
  NAIVE_PIPELINE(1);

  private final int index;

  VisionPipeline(int index) {
    this.index = index;
  }

  public int getIndex() {
    return index;
  }
}
