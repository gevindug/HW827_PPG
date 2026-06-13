#include <HW827_PPG.h>

// HW-827 signal pin
const int PPG_PIN = A0;

HW827_PPG ppg(PPG_PIN);

void setup() {
  Serial.begin(115200);
  ppg.begin();

  // Optional tuning
  ppg.setSampleInterval(20);       // 50 Hz
  ppg.setBpmSmoothing(0.15);       // lower = smoother BPM
  ppg.setMaxIbiChange(0.25);       // reject sudden interval changes above 25 percent
}

void loop() {
  if (ppg.update()) {
    // Serial Plotter columns:
    // raw, filtered, differentiated, top marker, bottom marker, bpm
    Serial.print(ppg.getRaw());
    Serial.print(",");
    Serial.print(ppg.getFiltered());
    Serial.print(",");
    Serial.print(ppg.getDiff() * 10.0);  // scaled only for plotting
    Serial.print(",");
    Serial.print(ppg.getTopMarker());
    Serial.print(",");
    Serial.print(ppg.getBottomMarker());
    Serial.print(",");
    Serial.println(ppg.getBPM());
  }
}
