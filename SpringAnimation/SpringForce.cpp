//
// Created by ktiays on 2023/1/18.
// Copyright (c) 2023 ktiays. All rights reserved.
//

#include <cmath>
#include <limits>
#include <algorithm>

#include "SpringForce.h"

#define UNSET std::numeric_limits<double>::infinity()

float SpringForce::StiffnessConstant::high = 1e4;
float SpringForce::StiffnessConstant::medium = 1500.f;
float SpringForce::StiffnessConstant::low = 200.f;
float SpringForce::StiffnessConstant::very_low = 50.f;

float SpringForce::DampingRatioConstant::high_bouncy = 0.2;
float SpringForce::DampingRatioConstant::medium_bouncy = 0.5;
float SpringForce::DampingRatioConstant::low_bouncy = 0.75;
float SpringForce::DampingRatioConstant::no_bouncy = 1.f;

SpringForce::SpringForce()
    : SpringForce(UNSET) {}

SpringForce::SpringForce(float final_position)
    : natural_freq_(std::sqrt(StiffnessConstant::medium)),
      damping_ratio_(DampingRatioConstant::medium_bouncy),
      state_({ 0, 0 }),
      final_position(final_position) {}

SpringForce &SpringForce::set_stiffness(float stiffness) {
    natural_freq_ = std::sqrt(std::max(0.f, stiffness));
    initialized_ = false;
    return *this;
}

SpringForce &SpringForce::set_damping_ratio(float damping_ratio) {
    damping_ratio_ = std::max(0.f, damping_ratio);
    initialized_ = false;
    return *this;
}

float SpringForce::acceleration(float last_displacement, float last_velocity) const {
    last_displacement -= static_cast<float>(final_position);

    const double k = natural_freq_ * natural_freq_;
    const double c = 2 * natural_freq_ * damping_ratio_;

    return static_cast<float>(-k * last_displacement - c * last_velocity);
}

bool SpringForce::is_at_equilibrium(float value, float velocity) const {
    return std::abs(velocity) < velocity_threshold_ &&
           std::abs(value - final_position) < value_threshold_;
}

void SpringForce::initialize() {
    if (initialized_) { return; }

    if (final_position == UNSET) {
        assert(false);
        return;
    }

    if (damping_ratio_ > 1) {
        // Over damping
        gamma_plus_ = -damping_ratio_ * natural_freq_
                      + natural_freq_ * std::sqrt(damping_ratio_ * damping_ratio_ - 1);
        gamma_minus_ = -damping_ratio_ * natural_freq_
                       - natural_freq_ * std::sqrt(damping_ratio_ * damping_ratio_ - 1);
    } else if (damping_ratio_ >= 0 && damping_ratio_ < 1) {
        // Under damping
        damped_freq_ = natural_freq_ * std::sqrt(1 - damping_ratio_ * damping_ratio_);
    }

    initialized_ = true;
}

SpringForce::MassState SpringForce::update_values(double last_displacement, double last_velocity, long time_elapsed) {
    initialize();

    const double delta_t = static_cast<double>(time_elapsed) / 1000.f;
    last_displacement -= final_position;
    double displacement;
    double current_velocity;
    if (damping_ratio_ > 1) {
        // Over damped
        const double coefficient_a = last_displacement - (gamma_minus_ * last_displacement - last_velocity)
                                                         / (gamma_minus_ - gamma_plus_);
        const double coefficient_b = (gamma_minus_ * last_displacement - last_velocity)
                                     / (gamma_minus_ - gamma_plus_);
        displacement = coefficient_a * std::exp(gamma_minus_ * delta_t)
                       + coefficient_b * std::exp(gamma_plus_ * delta_t);
        current_velocity = coefficient_a * gamma_minus_ * std::exp(gamma_minus_ * delta_t)
                           + coefficient_b * gamma_plus_ * std::exp(gamma_plus_ * delta_t);
    } else if (damping_ratio_ == 1) {
        // Critically damped
        const double coefficient_a = last_displacement;
        const double coefficient_b = last_velocity + natural_freq_ * last_displacement;
        displacement = (coefficient_a + coefficient_b * delta_t) * std::exp(-natural_freq_ * delta_t);
        current_velocity = (coefficient_a + coefficient_b * delta_t) * std::exp(-natural_freq_ * delta_t)
                           * (-natural_freq_) + coefficient_b * std::exp(-natural_freq_ * delta_t);
    } else {
        // Under damped
        double cos_coefficient = last_displacement;
        double sin_coefficient = (1 / damped_freq_) * (damping_ratio_ * natural_freq_
                                                       * last_displacement + last_velocity);
        displacement = std::exp(-damping_ratio_ * natural_freq_ * delta_t)
                       * (cos_coefficient * std::cos(damped_freq_ * delta_t)
                          + sin_coefficient * std::sin(damped_freq_ * delta_t));
        current_velocity = displacement * (-natural_freq_) * damping_ratio_
                           + std::exp(-damping_ratio_ * natural_freq_ * delta_t)
                             * (-damped_freq_ * cos_coefficient * std::sin(damped_freq_ * delta_t)
                                + damped_freq_ * sin_coefficient * std::cos(damped_freq_ * delta_t));
    }
    state_.value = static_cast<float>(displacement + final_position);
    state_.velocity = static_cast<float>(current_velocity);
    return state_;
}

SpringForce &SpringForce::set_value_threshold(double threshold) {
    value_threshold_ = std::abs(threshold);
    velocity_threshold_ = value_threshold_ * velocity_threshold_multiplier_;
    return *this;
}