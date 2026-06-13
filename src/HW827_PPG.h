#ifndef HW827_PPG_H
#define HW827_PPG_H

#include <Arduino.h>

class HW827_PPG {
public:
  explicit HW827_PPG(int analogPin);

  void begin();

  // Call this as fast as possible inside loop().
  // It returns true only when a new sample has been processed.
  bool update();

  // Use this if you read ADC yourself. It processes one raw sample immediately.
  // It returns true when processing is complete.
  bool updateWithRaw(int rawValue);

  // Reset BPM and beat tracking, but keep filter states.
  void resetHeartRate();

  // Reset all filter and heart-rate states.
  void resetAll();

  // Configuration
  void setSampleInterval(unsigned long sampleMs);
  void setBaselineAlpha(float alpha);
  void setLowpassAlpha(float alpha1, float alpha2);
  void setDerivativeAlpha(float alpha);
  void setBpmSmoothing(float smoothing);
  void setMaxIbiChange(float fraction);
  void setMinPulseAmplitude(float amplitude);
  void setIbiLimits(unsigned long minIbiMs, unsigned long maxIbiMs);

  // Output getters
  int getRaw() const;
  int getMedianRaw() const;
  float getBaseline() const;
  float getFiltered() const;
  float getDiff() const;
  float getBPM() const;
  float getTopMarker() const;
  float getBottomMarker() const;
  float getMinPulseAmplitude() const;

  bool beatDetected() const;
  bool topDetected() const;
  bool bottomDetected() const;

private:
  static const int MED_SIZE = 5;
  static const int MA_SIZE = 5;
  static const int IBI_SIZE = 7;

  int _pin;
  unsigned long _sampleMs;
  unsigned long _lastSample;

  // Raw and filtering
  int _raw;
  int _medianRaw;
  int _medBuf[MED_SIZE];
  int _medIndex;

  float _baseline;
  float _hp;
  float _lp1;
  float _lp2;

  float _baselineAlpha;
  float _lpAlpha1;
  float _lpAlpha2;

  float _maBuf[MA_SIZE];
  int _maIndex;
  float _maSum;

  float _filtered;

  // Derivative
  float _prevPPG;
  float _diff;
  float _diffSmooth;
  float _prevDiffSmooth;
  float _diffAlpha;
  bool _firstSample;

  // Extrema / beat detection
  float _lastMaxVal;
  float _lastMinVal;
  bool _haveMax;
  bool _haveMin;

  unsigned long _lastMaxTime;
  unsigned long _lastMinTime;
  unsigned long _lastValidBeatTime;

  unsigned long _minIbi;
  unsigned long _maxIbi;

  float _minPulseAmp;

  bool _beatDetected;
  bool _topDetected;
  bool _bottomDetected;
  float _topMarker;
  float _bottomMarker;

  // BPM filter
  unsigned long _ibiBuf[IBI_SIZE];
  int _ibiIndex;
  int _ibiCount;

  float _bpm;
  float _filteredIBI;
  bool _haveFilteredIBI;

  float _maxIbiChange;
  float _bpmSmoothing;

  int median5(const int values[MED_SIZE]) const;
  unsigned long getMedianIBI() const;
  bool updateBPM(unsigned long ibi);
  bool processSample(int rawValue, unsigned long now);
};

#endif
