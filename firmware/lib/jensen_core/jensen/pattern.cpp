#include "jensen/pattern.h"

#include <cmath>

#include "jensen/util.h"

namespace jensen {

void PatternRecognizer::configure(const PatternConfig& config) {
    config_ = config;
    if (config_.siteCount < 3) config_.siteCount = 3;
    if (config_.siteCount > kPatternSiteCount) config_.siteCount = kPatternSiteCount;
    config_.signalThreshold = clamp01(config_.signalThreshold);
    config_.releaseThreshold = clamp(config_.releaseThreshold, 0.0f, config_.signalThreshold);
    config_.maximumDistance = clamp(config_.maximumDistance, 0.01f, 2.0f);
    config_.ambiguityMargin = clamp(config_.ambiguityMargin, 0.0f, 2.0f);
    config_.holdSeconds = clamp(config_.holdSeconds, 0.0f, 5.0f);
    config_.trainingSettleSeconds = clamp(config_.trainingSettleSeconds, 0.0f, 10.0f);
    config_.trainingCaptureSeconds = clamp(config_.trainingCaptureSeconds, 0.1f, 30.0f);
}

void PatternRecognizer::reset() {
    candidate_ = GripId::Rest;
    eventGrip_ = GripId::Rest;
    confidence_ = 0.0f;
    candidateTimer_ = 0.0f;
    armed_ = true;
    event_ = false;
    cancelTraining();
    trainingResult_ = false;
}

void PatternRecognizer::update(const float features[kPatternSiteCount], float dt) {
    if (dt <= 0.0f) return;
    if (training_) {
        updateTraining(features, dt);
        return;
    }
    if (!config_.enabled) return;

    float peak = 0.0f;
    for (uint8_t i = 0; i < config_.siteCount; ++i) peak = features[i] > peak ? features[i] : peak;
    if (peak <= config_.releaseThreshold) {
        armed_ = true;
        candidateTimer_ = 0.0f;
        confidence_ = 0.0f;
        candidate_ = GripId::Rest;
        return;
    }
    if (!armed_ || peak < config_.signalThreshold) {
        candidateTimer_ = 0.0f;
        return;
    }

    float best = 1e9f;
    float second = 1e9f;
    GripId bestGrip = GripId::Rest;
    bool found = false;
    for (uint8_t g = 0; g < kGripCount; ++g) {
        const PatternTemplate& pattern = config_.templates[g];
        if (!pattern.valid) continue;
        float squared = 0.0f;
        for (uint8_t i = 0; i < config_.siteCount; ++i) {
            const float delta = clamp01(features[i]) - clamp01(pattern.centroid[i]);
            squared += delta * delta;
        }
        const float distance = std::sqrt(squared / static_cast<float>(config_.siteCount));
        if (distance < best) {
            second = best;
            best = distance;
            bestGrip = static_cast<GripId>(g);
            found = true;
        } else if (distance < second) {
            second = distance;
        }
    }

    const bool distanceOk = found && best <= config_.maximumDistance;
    const bool marginOk = second >= 1e8f || (second - best) >= config_.ambiguityMargin;
    confidence_ = distanceOk ? clamp01(1.0f - best / config_.maximumDistance) : 0.0f;
    if (!distanceOk || !marginOk) {
        candidateTimer_ = 0.0f;
        candidate_ = GripId::Rest;
        return;
    }

    if (candidate_ != bestGrip) {
        candidate_ = bestGrip;
        candidateTimer_ = 0.0f;
    } else {
        candidateTimer_ += dt;
    }
    if (candidateTimer_ >= config_.holdSeconds) {
        eventGrip_ = bestGrip;
        event_ = true;
        armed_ = false;
        candidateTimer_ = 0.0f;
    }
}

bool PatternRecognizer::consumeGrip(GripId& grip) {
    if (!event_) return false;
    grip = eventGrip_;
    event_ = false;
    return true;
}

bool PatternRecognizer::beginTraining(GripId grip) {
    if (static_cast<uint8_t>(grip) >= kGripCount || grip == GripId::Rest) return false;
    training_ = true;
    trainingResult_ = false;
    trainingGrip_ = grip;
    trainingTimer_ = 0.0f;
    trainingSamples_ = 0;
    for (double& sum : trainingSum_) sum = 0.0;
    return true;
}

void PatternRecognizer::cancelTraining() {
    training_ = false;
    trainingTimer_ = 0.0f;
    trainingSamples_ = 0;
}

float PatternRecognizer::trainingProgress() const {
    if (!training_) return trainingResult_ ? 1.0f : 0.0f;
    const float total = config_.trainingSettleSeconds + config_.trainingCaptureSeconds;
    return total > 0.0f ? clamp01(trainingTimer_ / total) : 1.0f;
}

void PatternRecognizer::updateTraining(const float features[kPatternSiteCount], float dt) {
    trainingTimer_ += dt;
    if (trainingTimer_ > config_.trainingSettleSeconds) {
        for (uint8_t i = 0; i < config_.siteCount; ++i) trainingSum_[i] += clamp01(features[i]);
        ++trainingSamples_;
    }
    const float total = config_.trainingSettleSeconds + config_.trainingCaptureSeconds;
    if (trainingTimer_ < total) return;

    PatternTemplate& pattern = config_.templates[static_cast<uint8_t>(trainingGrip_)];
    float peak = 0.0f;
    for (uint8_t i = 0; i < config_.siteCount; ++i) {
        pattern.centroid[i] = trainingSamples_ > 0
                                  ? static_cast<float>(trainingSum_[i] / trainingSamples_)
                                  : 0.0f;
        peak = pattern.centroid[i] > peak ? pattern.centroid[i] : peak;
    }
    pattern.valid = trainingSamples_ > 0 && peak >= config_.signalThreshold;
    training_ = false;
    trainingResult_ = pattern.valid;
}

bool PatternRecognizer::consumeTrainingResult(PatternConfig& config) {
    if (!trainingResult_) return false;
    config = config_;
    trainingResult_ = false;
    return true;
}

}  // namespace jensen
