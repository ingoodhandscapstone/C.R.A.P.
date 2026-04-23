#ifndef BLOOD_OXYGEN_PROCESSOR_H
#define BLOOD_OXYGEN_PROCESSOR_H

#include <cstdint>

class BloodOxygenProcessor {
    static const int FreqS = 25;
    static const int BUFFER_SIZE = FreqS * 4;
    static const int MA4_SIZE = 4;

    static const uint8_t uch_spo2_table[184];

    uint32_t irBuffer[BUFFER_SIZE];
    uint32_t redBuffer[BUFFER_SIZE];
    int sampleCount;
    int writeIndex;
    int32_t lastSpo2;
    int8_t lastSpo2Valid;
    int32_t lastHeartRate;
    int8_t lastHeartRateValid;

    void addSample(uint32_t ir, uint32_t red);
    void buildOrderedWindow(uint32_t *irWindow, uint32_t *redWindow) const;

    void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,
                                                int32_t n_ir_buffer_length,
                                                uint32_t *pun_red_buffer,
                                                int32_t *pn_spo2,
                                                int8_t *pch_spo2_valid,
                                                int32_t *pn_heart_rate,
                                                int8_t *pch_hr_valid);
    void maxim_find_peaks(int32_t *pn_locs,
                          int32_t *n_npks,
                          int32_t *pn_x,
                          int32_t n_size,
                          int32_t n_min_height,
                          int32_t n_min_distance,
                          int32_t n_max_num);
    void maxim_peaks_above_min_height(int32_t *pn_locs,
                                      int32_t *n_npks,
                                      int32_t *pn_x,
                                      int32_t n_size,
                                      int32_t n_min_height);
    void maxim_remove_close_peaks(int32_t *pn_locs,
                                  int32_t *pn_npks,
                                  int32_t *pn_x,
                                  int32_t n_min_distance);
    void maxim_sort_ascend(int32_t *pn_x, int32_t n_size);
    void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size);

  public:
    BloodOxygenProcessor()
        : irBuffer{0},
          redBuffer{0},
          sampleCount(0),
          writeIndex(0),
          lastSpo2(-999),
          lastSpo2Valid(0),
          lastHeartRate(-999),
          lastHeartRateValid(0) {}

    // Returns true once conversion takes place which requires a certain amount of samples
    bool getSPO2(int& spo2, int& ir, int& red);

    void reset();
};

#endif
