#include "HW827_PPG.h"

HW827_PPG::HW827_PPG(int analogPin) {
  _pin = analogPin;
  _sampleMs = 20;          // 50 Hz
  _lastSample = 0;

  _raw = 0;
  _medianRaw = 0;
  _medIndex = 0;

  _baseline = 512.0;
  _hp = 0.0;
  _lp1 = 0.0;
  _lp2 = 0.0;

  _baselineAlpha = 0.0015;
  _lpAlpha1 = 0.18;
  _lpAlpha2 = 0.08;

  _maIndex = 0;
  _maSum = 0.0;
  _filtered = 0.0;

  _prevPPG = 0.0;
  _diff = 0.0;
  _diffSmooth = 0.0;
  _prevDiffSmooth = 0.0;
  _diffAlpha = 0.45;
  _firstSample = true;

  _lastMaxVal = 0.0;
  _lastMinVal = 0.0;
  _haveMax = false;
  _haveMin = false;

  _lastMaxTime = 0;
  _lastMinTime = 0;
  _lastValidBeatTime = 0;

  _minIbi = 300;           // 200 BPM max
  _maxIbi = 2000;          // 30 BPM min
  _minPulseAmp = 0.15;

  _beatDetected = false;
  _topDetected = false;
  _bottomDetected = false;
  _topMarker = 0.0;
  _bottomMarker = 0.0;

  _ibiIndex = 0;
  _ibiCount = 0;
  _bpm = 0.0;
  _filteredIBI = 0.0;
  _haveFilteredIBI = false;

  _maxIbiChange = 0.25;
  _bpmSmoothing = 0.15;

  for (int i = 0; i < MED_SIZE; i++) {
    _medBuf[i] = 512;
  }
  for (int i = 0; i < MA_SIZE; i++) {
    _maBuf[i] = 0.0;
  }
  for (int i = 0; i < IBI_SIZE; i++) {
    _ibiBuf[i] = 0;
  }
}

void HW827_PPG::begin() {
  pinMode(_pin, INPUT);

  int initial = analogRead(_pin);
  _baseline = (float)initial;
  _raw = initial;
  _medianRaw = initial;

  for (int i = 0; i < MED_SIZE; i++) {
    _medBuf[i] = initial;
  }

  for (int i = 0; i < MA_SIZE; i++) {
    _maBuf[i] = 0.0;
  }

  _maSum = 0.0;
  _lastSample = millis();
}

bool HW827_PPG::update() {
  unsigned long now = millis();

  if (now - _lastSample < _sampleMs) {
    return false;
  }

  _lastSample = now;
  int rawValue = analogRead(_pin);
  return processSample(rawValue, now);
}

bool HW827_PPG::updateWithRaw(int rawValue) {
  unsigned long now = millis();
  return processSample(rawValue, now);
}

bool HW827_PPG::processSample(int rawValue, unsigned long now) {
  _beatDetected = false;
  _topDetected = false;
  _bottomDetected = false;
  _topMarker = 0.0;
  _bottomMarker = 0.0;

  _raw = rawValue;

  // Median filter for spike removal
  _medBuf[_medIndex] = _raw;
  _medIndex++;
  if (_medIndex >= MED_SIZE) {
    _medIndex = 0;
  }

  _medianRaw = median5(_medBuf);

  // Baseline correction
  _baseline = _baseline + _baselineAlpha * ((float)_medianRaw - _baseline);
  _hp = (float)_medianRaw - _baseline;

  // Low-pass smoothing
  _lp1 = _lp1 + _lpAlpha1 * (_hp - _lp1);
  _lp2 = _lp2 + _lpAlpha2 * (_lp1 - _lp2);

  // Moving average smoothing
  _maSum = _maSum - _maBuf[_maIndex];
  _maBuf[_maIndex] = _lp2;
  _maSum = _maSum + _maBuf[_maIndex];

  _maIndex++;
  if (_maIndex >= MA_SIZE) {
    _maIndex = 0;
  }

  _filtered = _maSum / (float)MA_SIZE;

  if (_firstSample) {
    _prevPPG = _filtered;
    _firstSample = false;
    return true;
  }

  // Differentiation and smoothing
  _diff = _filtered - _prevPPG;
  _diffSmooth = _diffSmooth + _diffAlpha * (_diff - _diffSmooth);

  // Pulse top: derivative changes positive to negative
  if (_prevDiffSmooth > 0.0 && _diffSmooth <= 0.0) {
    float currentMax = _prevPPG;
    unsigned long currentMaxTime = now - _sampleMs;

    _haveMax = true;
    _lastMaxVal = currentMax;

    if (_haveMin) {
      float pulseAmp = currentMax - _lastMinVal;

      if (pulseAmp > _minPulseAmp) {
        if (_lastMaxTime > 0) {
          unsigned long ibi = currentMaxTime - _lastMaxTime;

          if (updateBPM(ibi)) {
            _topMarker = currentMax + 0.8;
            _topDetected = true;
            _beatDetected = true;
            _lastValidBeatTime = now;
          }
        }

        _lastMaxTime = currentMaxTime;

        _minPulseAmp = 0.85 * _minPulseAmp + 0.15 * (pulseAmp * 0.25);
        if (_minPulseAmp < 0.08) _minPulseAmp = 0.08;
        if (_minPulseAmp > 3.00) _minPulseAmp = 3.00;
      }
    }
  }

  // Inverted pulse bottom: derivative changes negative to positive
  if (_prevDiffSmooth < 0.0 && _diffSmooth >= 0.0) {
    float currentMin = _prevPPG;
    unsigned long currentMinTime = now - _sampleMs;

    _haveMin = true;
    _lastMinVal = currentMin;

    if (_haveMax) {
      float pulseAmp = _lastMaxVal - currentMin;

      if (pulseAmp > _minPulseAmp) {
        if (_lastMinTime > 0) {
          unsigned long ibi = currentMinTime - _lastMinTime;

          if (updateBPM(ibi)) {
            _bottomMarker = currentMin - 0.8;
            _bottomDetected = true;
            _beatDetected = true;
            _lastValidBeatTime = now;
          }
        }

        _lastMinTime = currentMinTime;

        _minPulseAmp = 0.85 * _minPulseAmp + 0.15 * (pulseAmp * 0.25);
        if (_minPulseAmp < 0.08) _minPulseAmp = 0.08;
        if (_minPulseAmp > 3.00) _minPulseAmp = 3.00;
      }
    }
  }

  // Reset BPM if no valid beat for too long
  if (_lastValidBeatTime > 0 && now - _lastValidBeatTime > 4000) {
    resetHeartRate();
  }

  _prevPPG = _filtered;
  _prevDiffSmooth = _diffSmooth;

  return true;
}

bool HW827_PPG::updateBPM(unsigned long ibi) {
  if (ibi < _minIbi || ibi > _maxIbi) {
    return false;
  }

  if (_haveFilteredIBI) {
    float diff = abs((float)ibi - _filteredIBI);
    float allowedDiff = _filteredIBI * _maxIbiChange;

    if (diff > allowedDiff) {
      return false;
    }
  }

  _ibiBuf[_ibiIndex] = ibi;
  _ibiIndex++;
  if (_ibiIndex >= IBI_SIZE) {
    _ibiIndex = 0;
  }

  if (_ibiCount < IBI_SIZE) {
    _ibiCount++;
  }

  unsigned long medianIBI = getMedianIBI();

  if (!_haveFilteredIBI) {
    _filteredIBI = (float)medianIBI;
    _haveFilteredIBI = true;
  } else {
    _filteredIBI = _filteredIBI + _bpmSmoothing * ((float)medianIBI - _filteredIBI);
  }

  _bpm = 60000.0 / _filteredIBI;
  return true;
}

void HW827_PPG::resetHeartRate() {
  _bpm = 0.0;
  _filteredIBI = 0.0;
  _haveFilteredIBI = false;

  _ibiIndex = 0;
  _ibiCount = 0;

  _lastMaxTime = 0;
  _lastMinTime = 0;
  _lastValidBeatTime = 0;

  _haveMax = false;
  _haveMin = false;

  _beatDetected = false;
  _topDetected = false;
  _bottomDetected = false;
  _topMarker = 0.0;
  _bottomMarker = 0.0;

  _minPulseAmp = 0.15;

  for (int i = 0; i < IBI_SIZE; i++) {
    _ibiBuf[i] = 0;
  }
}

void HW827_PPG::resetAll() {
  resetHeartRate();

  _hp = 0.0;
  _lp1 = 0.0;
  _lp2 = 0.0;
  _maSum = 0.0;
  _filtered = 0.0;
  _prevPPG = 0.0;
  _diff = 0.0;
  _diffSmooth = 0.0;
  _prevDiffSmooth = 0.0;
  _firstSample = true;

  for (int i = 0; i < MA_SIZE; i++) {
    _maBuf[i] = 0.0;
  }
}

void HW827_PPG::setSampleInterval(unsigned long sampleMs) {
  _sampleMs = sampleMs;
}

void HW827_PPG::setBaselineAlpha(float alpha) {
  _baselineAlpha = alpha;
}

void HW827_PPG::setLowpassAlpha(float alpha1, float alpha2) {
  _lpAlpha1 = alpha1;
  _lpAlpha2 = alpha2;
}

void HW827_PPG::setDerivativeAlpha(float alpha) {
  _diffAlpha = alpha;
}

void HW827_PPG::setBpmSmoothing(float smoothing) {
  _bpmSmoothing = smoothing;
}

void HW827_PPG::setMaxIbiChange(float fraction) {
  _maxIbiChange = fraction;
}

void HW827_PPG::setMinPulseAmplitude(float amplitude) {
  _minPulseAmp = amplitude;
}

void HW827_PPG::setIbiLimits(unsigned long minIbiMs, unsigned long maxIbiMs) {
  _minIbi = minIbiMs;
  _maxIbi = maxIbiMs;
}

int HW827_PPG::getRaw() const {
  return _raw;
}

int HW827_PPG::getMedianRaw() const {
  return _medianRaw;
}

float HW827_PPG::getBaseline() const {
  return _baseline;
}

float HW827_PPG::getFiltered() const {
  return _filtered;
}

float HW827_PPG::getDiff() const {
  return _diffSmooth;
}

float HW827_PPG::getBPM() const {
  return _bpm;
}

float HW827_PPG::getTopMarker() const {
  return _topMarker;
}

float HW827_PPG::getBottomMarker() const {
  return _bottomMarker;
}

float HW827_PPG::getMinPulseAmplitude() const {
  return _minPulseAmp;
}

bool HW827_PPG::beatDetected() const {
  return _beatDetected;
}

bool HW827_PPG::topDetected() const {
  return _topDetected;
}

bool HW827_PPG::bottomDetected() const {
  return _bottomDetected;
}

int HW827_PPG::median5(const int values[MED_SIZE]) const {
  int temp[MED_SIZE];

  for (int i = 0; i < MED_SIZE; i++) {
    temp[i] = values[i];
  }

  for (int i = 0; i < MED_SIZE - 1; i++) {
    for (int j = i + 1; j < MED_SIZE; j++) {
      if (temp[j] < temp[i]) {
        int t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  return temp[MED_SIZE / 2];
}

unsigned long HW827_PPG::getMedianIBI() const {
  unsigned long temp[IBI_SIZE];

  for (int i = 0; i < _ibiCount; i++) {
    temp[i] = _ibiBuf[i];
  }

  for (int i = 0; i < _ibiCount - 1; i++) {
    for (int j = i + 1; j < _ibiCount; j++) {
      if (temp[j] < temp[i]) {
        unsigned long t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  return temp[_ibiCount / 2];
}
