#include "BloodOxygenProcessor.h"

#include <algorithm>

const uint8_t BloodOxygenProcessor::uch_spo2_table[184] = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
    99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98,
    98, 98, 98, 97, 97, 97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93,
    92, 92, 92, 91, 91, 90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84,
    83, 82, 82, 81, 81, 80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71,
    70, 69, 69, 68, 67, 66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55,
    54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35,
    34, 33, 31, 30, 29, 28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11,
    10, 9, 7, 6, 5, 3, 2, 1
};

void BloodOxygenProcessor::addSample(uint32_t ir, uint32_t red) {
    irBuffer[writeIndex] = ir;
    redBuffer[writeIndex] = red;

    writeIndex++;
    if (writeIndex >= BUFFER_SIZE) {
        writeIndex = 0;
    }

    if (sampleCount < BUFFER_SIZE) {
        sampleCount++;
    }
}

void BloodOxygenProcessor::buildOrderedWindow(uint32_t *irWindow, uint32_t *redWindow) const {
    if (sampleCount < BUFFER_SIZE) {
        return;
    }

    for (int i = 0; i < BUFFER_SIZE; i++) {
        const int idx = (writeIndex + i) % BUFFER_SIZE;
        irWindow[i] = irBuffer[idx];
        redWindow[i] = redBuffer[idx];
    }
}

bool BloodOxygenProcessor::getSPO2(int& spo2, int& ir, int& red) {
    addSample(static_cast<uint32_t>(ir), static_cast<uint32_t>(red));

    if (sampleCount < BUFFER_SIZE) {
        return false;
    }

    uint32_t orderedIr[BUFFER_SIZE];
    uint32_t orderedRed[BUFFER_SIZE];
    buildOrderedWindow(orderedIr, orderedRed);

    maxim_heart_rate_and_oxygen_saturation(orderedIr,
                                           BUFFER_SIZE,
                                           orderedRed,
                                           &lastSpo2,
                                           &lastSpo2Valid,
                                           &lastHeartRate,
                                           &lastHeartRateValid);

    spo2 = static_cast<int>(lastSpo2);

    return lastSpo2Valid == 1;
}

void BloodOxygenProcessor::reset() {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        irBuffer[i] = 0;
        redBuffer[i] = 0;
    }

    sampleCount = 0;
    writeIndex = 0;
    lastSpo2 = -999;
    lastSpo2Valid = 0;
    lastHeartRate = -999;
    lastHeartRateValid = 0;
}


void BloodOxygenProcessor::maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,
                                                                   int32_t n_ir_buffer_length,
                                                                   uint32_t *pun_red_buffer,
                                                                   int32_t *pn_spo2,
                                                                   int8_t *pch_spo2_valid,
                                                                   int32_t *pn_heart_rate,
                                                                   int8_t *pch_hr_valid)
{
    uint32_t un_ir_mean;
    int32_t k, n_i_ratio_count;
    int32_t i, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks;
    int32_t an_ir_valley_locs[15];
    int32_t n_peak_interval_sum;

    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc;
    int32_t n_y_dc_max, n_x_dc_max;
    int32_t n_y_dc_max_idx = 0;
    int32_t n_x_dc_max_idx = 0;
    int32_t an_ratio[5], n_ratio_average;
    int32_t n_nume, n_denom;

    int32_t an_x[BUFFER_SIZE];
    int32_t an_y[BUFFER_SIZE];

    // calculates DC mean and subtract DC from ir
    un_ir_mean = 0;
    for (k = 0; k < n_ir_buffer_length; k++) {
        un_ir_mean += pun_ir_buffer[k];
    }
    un_ir_mean = un_ir_mean / n_ir_buffer_length;

    // remove DC and invert signal so that we can use peak detector as valley detector
    for (k = 0; k < n_ir_buffer_length; k++) {
        an_x[k] = -1 * (static_cast<int32_t>(pun_ir_buffer[k] - un_ir_mean));
    }

    // 4 pt Moving Average
    for (k = 0; k < BUFFER_SIZE - MA4_SIZE; k++) {
        an_x[k] = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / static_cast<int32_t>(4);
    }

    // calculate threshold
    n_th1 = 0;
    for (k = 0; k < BUFFER_SIZE; k++) {
        n_th1 += an_x[k];
    }
    n_th1 = n_th1 / BUFFER_SIZE;
    if (n_th1 < 30) {
        n_th1 = 30;
    }
    if (n_th1 > 60) {
        n_th1 = 60;
    }

    for (k = 0; k < 15; k++) {
        an_ir_valley_locs[k] = 0;
    }

    // since we flipped signal, we use peak detector as valley detector
    maxim_find_peaks(an_ir_valley_locs,
                     &n_npks,
                     an_x,
                     BUFFER_SIZE,
                     n_th1,
                     4,
                     15);

    n_peak_interval_sum = 0;
    if (n_npks >= 2) {
        for (k = 1; k < n_npks; k++) {
            n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k - 1]);
        }
        n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
        *pn_heart_rate = static_cast<int32_t>((FreqS * 60) / n_peak_interval_sum);
        *pch_hr_valid = 1;
    } else {
        *pn_heart_rate = -999;
        *pch_hr_valid = 0;
    }

    // load raw value again for SPO2 calculation : RED(=y) and IR(=X)
    for (k = 0; k < n_ir_buffer_length; k++) {
        an_x[k] = static_cast<int32_t>(pun_ir_buffer[k]);
        an_y[k] = static_cast<int32_t>(pun_red_buffer[k]);
    }

    // find precise min near an_ir_valley_locs
    n_exact_ir_valley_locs_count = n_npks;

    // using exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration an_ratio
    // finding AC/DC maximum of raw

    n_ratio_average = 0;
    n_i_ratio_count = 0;
    for (k = 0; k < 5; k++) {
        an_ratio[k] = 0;
    }

    for (k = 0; k < n_exact_ir_valley_locs_count; k++) {
        if (an_ir_valley_locs[k] > BUFFER_SIZE) {
            *pn_spo2 = -999;
            *pch_spo2_valid = 0;
            return;
        }
    }

    // find max between two valley locations
    // and use an_ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2
    for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++) {
        n_y_dc_max = -16777216;
        n_x_dc_max = -16777216;
        if (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k] > 3) {
            for (i = an_ir_valley_locs[k]; i < an_ir_valley_locs[k + 1]; i++) {
                if (an_x[i] > n_x_dc_max) {
                    n_x_dc_max = an_x[i];
                    n_x_dc_max_idx = i;
                }
                if (an_y[i] > n_y_dc_max) {
                    n_y_dc_max = an_y[i];
                    n_y_dc_max_idx = i;
                }
            }

            n_y_ac = (an_y[an_ir_valley_locs[k + 1]] - an_y[an_ir_valley_locs[k]]) *
                     (n_y_dc_max_idx - an_ir_valley_locs[k]);
            n_y_ac = an_y[an_ir_valley_locs[k]] +
                     n_y_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);
            n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;

            n_x_ac = (an_x[an_ir_valley_locs[k + 1]] - an_x[an_ir_valley_locs[k]]) *
                     (n_x_dc_max_idx - an_ir_valley_locs[k]);
            n_x_ac = an_x[an_ir_valley_locs[k]] +
                     n_x_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);
            n_x_ac = an_x[n_x_dc_max_idx] - n_x_ac;

            n_nume = (n_y_ac * n_x_dc_max) >> 7;
            n_denom = (n_x_ac * n_y_dc_max) >> 7;

            if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0) {
                an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom;
                n_i_ratio_count++;
            }
        }
    }

    // choose median value since PPG signal may varies from beat to beat
    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx = n_i_ratio_count / 2;

    if (n_middle_idx > 1) {
        n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2;
    } else {
        n_ratio_average = an_ratio[n_middle_idx];
    }

    if (n_ratio_average > 2 && n_ratio_average < 184) {
        n_spo2_calc = uch_spo2_table[n_ratio_average];
        *pn_spo2 = n_spo2_calc;
        *pch_spo2_valid = 1;
    } else {
        *pn_spo2 = -999;
        *pch_spo2_valid = 0;
    }
}


void BloodOxygenProcessor::maxim_find_peaks(int32_t *pn_locs,
                                            int32_t *n_npks,
                                            int32_t *pn_x,
                                            int32_t n_size,
                                            int32_t n_min_height,
                                            int32_t n_min_distance,
                                            int32_t n_max_num)
{
    maxim_peaks_above_min_height(pn_locs, n_npks, pn_x, n_size, n_min_height);
    maxim_remove_close_peaks(pn_locs, n_npks, pn_x, n_min_distance);
    *n_npks = std::min(*n_npks, n_max_num);
}

void BloodOxygenProcessor::maxim_peaks_above_min_height(int32_t *pn_locs,
                                                         int32_t *n_npks,
                                                         int32_t *pn_x,
                                                         int32_t n_size,
                                                         int32_t n_min_height)
{
    int32_t i = 1;
    int32_t n_width;
    *n_npks = 0;

    while (i < n_size - 1) {
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1]) {
            n_width = 1;
            while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width]) {
                n_width++;
            }
            if (pn_x[i] > pn_x[i + n_width] && (*n_npks) < 15) {
                pn_locs[(*n_npks)++] = i;
                i += n_width + 1;
            } else {
                i += n_width;
            }
        } else {
            i++;
        }
    }
}

void BloodOxygenProcessor::maxim_remove_close_peaks(int32_t *pn_locs,
                                                     int32_t *pn_npks,
                                                     int32_t *pn_x,
                                                     int32_t n_min_distance)
{
    int32_t i;
    int32_t j;
    int32_t n_old_npks;
    int32_t n_dist;

    maxim_sort_indices_descend(pn_x, pn_locs, *pn_npks);

    for (i = -1; i < *pn_npks; i++) {
        n_old_npks = *pn_npks;
        *pn_npks = i + 1;
        for (j = i + 1; j < n_old_npks; j++) {
            n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]);
            if (n_dist > n_min_distance || n_dist < -n_min_distance) {
                pn_locs[(*pn_npks)++] = pn_locs[j];
            }
        }
    }

    maxim_sort_ascend(pn_locs, *pn_npks);
}

void BloodOxygenProcessor::maxim_sort_ascend(int32_t *pn_x, int32_t n_size) {
    int32_t i;
    int32_t j;
    int32_t n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--) {
            pn_x[j] = pn_x[j - 1];
        }
        pn_x[j] = n_temp;
    }
}

void BloodOxygenProcessor::maxim_sort_indices_descend(int32_t *pn_x,
                                                       int32_t *pn_indx,
                                                       int32_t n_size)
{
    int32_t i;
    int32_t j;
    int32_t n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--) {
            pn_indx[j] = pn_indx[j - 1];
        }
        pn_indx[j] = n_temp;
    }
}
