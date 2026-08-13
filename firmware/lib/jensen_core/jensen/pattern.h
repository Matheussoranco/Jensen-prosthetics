// Jensen Hand - multi-site EMG pattern recognition for direct grip selection.
#pragma once

#include <cstdint>

#include "jensen/grip.h"

namespace jensen {

constexpr uint8_t kPatternSiteCount = 4;

struct PatternTemplate {
    float centroid[kPatternSiteCount] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool valid = false;
    uint8_t reserved[3] = {0, 0, 0};
};

struct PatternConfig {
    bool enabled = false;
    uint8_t siteCount = kPatternSiteCount;
    uint8_t reserved[2] = {0, 0};
    float signalThreshold = 0.28f;
    float releaseThreshold = 0.12f;
    float maximumDistance = 0.35f;
    float ambiguityMargin = 0.08f;
    float holdSeconds = 0.18f;
    float trainingSettleSeconds = 0.75f;
    float trainingCaptureSeconds = 1.50f;
    PatternTemplate templates[kGripCount]{};
};

/// Nearest-centroid classifier with rejection, dwell, and release-to-rearm.
/// Features are calibrated site activations in [0,1]. A recognized pattern is
/// a one-shot grip event; it cannot repeat until all sites relax.
class PatternRecognizer {
public:
    void configure(const PatternConfig& config);
    const PatternConfig& config() const { return config_; }
    void reset();

    void update(const float features[kPatternSiteCount], float dt);
    bool consumeGrip(GripId& grip);

    GripId candidate() const { return candidate_; }
    float confidence() const { return confidence_; }
    bool armed() const { return armed_; }

    bool beginTraining(GripId grip);
    void cancelTraining();
    bool training() const { return training_; }
    float trainingProgress() const;
    /// True once when a template has completed; copies the updated config.
    bool consumeTrainingResult(PatternConfig& config);

private:
    void updateTraining(const float features[kPatternSiteCount], float dt);

    PatternConfig config_{};
    GripId candidate_ = GripId::Rest;
    GripId eventGrip_ = GripId::Rest;
    float confidence_ = 0.0f;
    float candidateTimer_ = 0.0f;
    bool armed_ = true;
    bool event_ = false;

    bool training_ = false;
    bool trainingResult_ = false;
    GripId trainingGrip_ = GripId::Rest;
    float trainingTimer_ = 0.0f;
    double trainingSum_[kPatternSiteCount] = {0.0, 0.0, 0.0, 0.0};
    uint32_t trainingSamples_ = 0;
};

}  // namespace jensen
