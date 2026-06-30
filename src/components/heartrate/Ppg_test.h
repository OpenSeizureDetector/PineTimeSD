#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
// Note: Change internal define 'sqrt_internal sqrt' to
// 'sqrt_internal sqrtf' to save ~3KB of flash.
#define sqrt_internal sqrtf
#define FFT_SPEED_OVER_PRECISION
#include "libs/arduinoFFT/src/arduinoFFT.h"

namespace Pinetime {
  namespace Controllers {
    class Ppg {
    public:
      Ppg();
      int8_t Preprocess(uint32_t hrs, uint32_t als);
      int HeartRate();
      void Reset(bool resetDaqBuffer);
      static constexpr int deltaTms = 100;
      // Daq dataLength: Must be power of 2
      static constexpr uint16_t dataLength = 64;
      static constexpr uint16_t spectrumLength = dataLength >> 1;

    private:
      // The sampling frequency (Hz) based on sampling time in milliseconds (DeltaTms)
      static constexpr float sampleFreq = 1000.0f / static_cast<float>(deltaTms);
      // The frequency resolution (Hz)
      static constexpr float freqResolution = sampleFreq / dataLength;
      // Number of samples before each analysis
      // 0.5 second update rate at 10Hz
      static constexpr uint16_t overlapWindow = 5;
      // Maximum number of spectrum running averages
      // Note: actual number of spectra averaged = spectralAvgMax + 1
      static constexpr uint16_t spectralAvgMax = 2;
      // Multiple Peaks above this threshold (% of max) are rejected
      static constexpr float peakDetectionThreshold = 0.6f;
      // Maximum peak width (bins) at threshold for valid peak.
      static constexpr float maxPeakWidth = 2.5f;
      // Metric for spectrum noise level.
      static constexpr float signalToNoiseThreshold = 3.0f;
      // Heart rate Region Of Interest begin (bins)
      static constexpr uint16_t hrROIbegin = static_cast<uint16_t>((30.0f / 60.0f) / freqResolution + 0.5f);
      // Heart rate Region Of Interest end (bins)
      static constexpr uint16_t hrROIend = static_cast<uint16_t>((240.0f / 60.0f) / freqResolution + 0.5f);
      // Minimum HR (Hz)
      static constexpr float minHR = 40.0f / 60.0f;
      // Maximum HR (Hz)
      static constexpr float maxHR = 230.0f / 60.0f;
      // Threshold for high DC level after filtering
      static constexpr float dcThreshold = 0.5f;
      // ALS detection factor
      static constexpr float alsFactor = 2.0f;

      // #### ALNF FREQUENCY TRACKING CONSTANTS ####
      // Forgetting factor for the autocorrelation tracking
      static constexpr float alnfLambda = 0.99f; 

      // Smoothing factor for the frequency estimation 
      static constexpr float alnfGamma = 0.98f;

      // Pole-zero contraction factor for the 
      static constexpr float alnfAlpha = 0.90f; 

      // #### COMB FILTER CONSTANTS ####
      // Number of harmonic notches to apply. (3-4 is good from paper)
      static constexpr int combN = 3;

      // #### ALNF STATE VARIABLES ####
      float g_t1 = 0.0f; // Delayed signal state g(t-1)
      float g_t2 = 0.0f; // Delayed signal state g(t-2)
      float Rg_0 = 1.0f; // Autocorrelation estimate (Rg(0)) init to 1.0 to prevent division by zero
      float Rg_1 = 0.0f; // Autocorrelation estimate (Rg(1))
      float k0_hat = 0.0f; // Smoothed adaptation parameter (tracks the HR frequency)


      // #### COMB FILTER STATE VARIABLES ####
      std::array<float, combN> comb_w1 = {0.0f, 0.0f, 0.0f};
      std::array<float, combN> comb_w2 = {0.0f, 0.0f, 0.0f};

      // Pole-zero contraction factor for the comb filter notches (controls width of harmonic filtering)
      static constexpr float combAlpha = 0.95f;
      
      // Raw ADC data
      std::array<uint16_t, dataLength> dataHRS;
      // Stores Real numbers from FFT
      std::array<float, dataLength> vReal;
      // Stores Imaginary numbers from FFT
      std::array<float, dataLength> vImag;
      // Stores power spectrum calculated from FFT real and imag values
      std::array<float, (spectrumLength)> spectrum;
      // Stores each new HR value (Hz). Non zero values are averaged for HR output
      std::array<float, 20> dataAverage;

      uint16_t avgIndex = 0;
      uint16_t spectralAvgCount = 0;
      float lastPeakLocation = 0.0f;
      uint16_t alsThreshold = UINT16_MAX;
      uint16_t alsValue = 0;
      uint16_t dataIndex = 0;
      float peakLocation;
      bool resetSpectralAvg = true;

      int ProcessHeartRate(bool init);
      float HeartRateAverage(float hr);
      void SpectrumAverage(const float* data, float* spectrum, int length, bool reset);

      // update HR estimation functions (Zainab)
      float ProcessALNF(float input);
      void ApplyCombFilter(std::array<float, Ppg::dataLength>& data, float estimatedFreq);
    };
  }
}
