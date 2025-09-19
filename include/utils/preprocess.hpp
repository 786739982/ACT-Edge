#include "dnn/hb_dnn.h"
#include <algorithm>
#include <queue>
#include <vector>
#include <cmath>

std::vector<double> pre_process(const std::vector<double>& s_qpos, const std::vector<double>& qpos_mean, const std::vector<double>& qpos_std) {
        size_t size = qpos_mean.size();
        std::vector<double> result(size);
        std::transform(s_qpos.begin(), s_qpos.end(), qpos_mean.begin(), result.begin(), [](double s, double m) {
            return (s - m);
        });
        std::transform(result.begin(), result.end(), qpos_std.begin(), result.begin(), [](double r, double std) {
            return r / std;
        });
        return result;
    }

bool compareDoubleArrays(const std::vector<double>& arr1, const std::vector<double>& arr2) {
    int size = arr1.size();
    for (int i = 0; i < size; ++i) {
        double diff = std::abs(arr1[i] - arr2[i]);
        if (diff >= 0.02) {
            return false;
        }
    }
    return true;
}