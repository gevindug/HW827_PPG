# HW827_PPG Arduino Library

Arduino library for the HW-827 analog pulse/PPG sensor.

It provides:

- Raw ADC signal
- Median-filtered raw signal
- Baseline-corrected and smoothed PPG signal
- Smoothed differentiated signal
- Pulse top and inverted pulse bottom markers
- Filtered BPM with abnormal reading rejection

## Installation

1. Download the ZIP file.
2. Open Arduino IDE.
3. Go to Sketch > Include Library > Add .ZIP Library.
4. Select the ZIP file.
5. Open File > Examples > HW827_PPG > SerialPlotter.

## Wiring

HW-827 signal pin -> A0
VCC -> 3.3 V or 5 V
GND -> GND

## Basic usage

```cpp
#include <HW827_PPG.h>

HW827_PPG ppg(A0);

void setup() {
  Serial.begin(115200);
  ppg.begin();
}

void loop() {
  if (ppg.update()) {
    Serial.print(ppg.getRaw());
    Serial.print(",");
    Serial.print(ppg.getFiltered());
    Serial.print(",");
    Serial.print(ppg.getDiff());
    Serial.print(",");
    Serial.println(ppg.getBPM());
  }
}
```

## Tuning

```cpp
ppg.setSampleInterval(20);       // 50 Hz
ppg.setBaselineAlpha(0.0015);    // smaller = slower baseline tracking
ppg.setLowpassAlpha(0.18, 0.08); // lower = smoother
ppg.setDerivativeAlpha(0.45);    // derivative smoothing
ppg.setBpmSmoothing(0.15);       // lower = smoother BPM
ppg.setMaxIbiChange(0.25);       // reject abnormal beat intervals
```

## Serial Plotter

The example prints:

```text
raw, filtered, differentiated, top_marker, bottom_marker, bpm
```

Use Arduino Serial Plotter at 115200 baud.
