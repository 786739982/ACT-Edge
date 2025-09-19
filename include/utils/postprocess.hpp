#include "dnn/hb_dnn.h"
#include <algorithm>
#include <queue>
#include <vector>

typedef struct Classification {
    int id;
    float score;
    const char* class_name;

    Classification()
        : class_name(0)
        , id(0)
        , score(0.0)
    {
    }
    Classification(int id, float score, const char* class_name)
        : id(id)
        , score(score)
        , class_name(class_name)
    {
    }

    friend bool operator>(const Classification& lhs, const Classification& rhs)
    {
        return (lhs.score > rhs.score);
    }

    ~Classification() { }
} Classification;

inline void get_topk_result(hbDNNTensor* tensor,
    std::vector<Classification>& top_k_cls,
    int top_k)
{
    hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    std::priority_queue<Classification,
        std::vector<Classification>,
        std::greater<Classification>>
        queue;
    int* shape = tensor->properties.validShape.dimensionSize;
    // The type reinterpret_cast should be determined according to the output type
    // For example: HB_DNN_TENSOR_TYPE_F32 is float
    auto data = reinterpret_cast<float*>(tensor->sysMem[0].virAddr);
    auto shift = tensor->properties.shift.shiftData;
    auto scale = tensor->properties.scale.scaleData;
    int tensor_len = shape[0] * shape[1] * shape[2] * shape[3];
    for (auto i = 0; i < tensor_len; i++) {
        float score = 0.0;
        if (tensor->properties.quantiType == SHIFT) {
            score = data[i] / (1 << shift[i]);
        } else if (tensor->properties.quantiType == SCALE) {
            score = data[i] * scale[i];
        } else {
            score = data[i];
        }
        queue.push(Classification(i, score, ""));
        if (queue.size() > top_k) {
            queue.pop();
        }
    }
    while (!queue.empty()) {
        top_k_cls.emplace_back(queue.top());
        queue.pop();
    }
    std::reverse(top_k_cls.begin(), top_k_cls.end());
}

std::vector<double> smooth_joints(const std::vector<std::vector<std::vector<double>>>& all_time_actions_,
    int t,
    int chunk_size,
    int action_dim)
{
    // Populate all_time_actions_ at timestep t with raw_actions
    std::vector<std::vector<double>> populated_actions;
    int bias_i = t - chunk_size + 1;
    int start_i = std::max(0, bias_i);
    int end_i = start_i + chunk_size + std::min(bias_i, 0);
    for (int i = start_i; i < end_i; ++i) {
        populated_actions.push_back(all_time_actions_[i][t]);
    }
    // Compute exponential weights
    double k = 0.01;
    int len = populated_actions.size();
    std::vector<double> exp_weights(len);
    for (int i = 0; i < len; ++i) {
        exp_weights[i] = std::exp(-k * i);
    }
    // Normalize weights
    double sum_weights = std::accumulate(exp_weights.begin(), exp_weights.end(), 0.0);
    for (double& weight : exp_weights) {
        weight /= sum_weights;
    }
    // Compute new_action
    std::vector<double> new_action(action_dim, 0.0);
    for (int i = 0; i < len; ++i) {
        for (int j = 0; j < action_dim; ++j) {
            new_action[j] += populated_actions[i][j] * exp_weights[i];
        }
    }

    return new_action;
}

std::vector<double> post_process(const std::vector<double>& a, const std::vector<double>& action_mean, const std::vector<double>& action_std) {
        std::vector<double> result(a.size());
        std::transform(a.begin(), a.end(), action_std.begin(), result.begin(), [](double a, double std) {
            return a * std;
        });
        std::transform(result.begin(), result.end(), action_mean.begin(), result.begin(), [](double r, double m) {
            return r + m;
        });
        return result;
}