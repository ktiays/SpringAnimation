//
// Created by ktiays on 2023/1/18.
// Copyright (c) 2023 ktiays. All rights reserved.
//

#include <utility>
#include <algorithm>

#include "SpringAnimation.h"

#define UNSET std::numeric_limits<float>::infinity()

SpringAnimation::SpringAnimation()
    : SpringAnimation(
    SpringForce()
        .set_value_threshold(0.75f)
) {}

SpringAnimation::SpringAnimation(float final_position)
    : SpringAnimation(
    SpringForce(final_position)
        .set_value_threshold(0.75f)
) {}

SpringAnimation::SpringAnimation(const SpringForce &spring_force)
    : spring_(spring_force),
      pending_position_(UNSET),
      value_(UNSET),
      min_value(-UNSET),
      max_value(UNSET) {}

SpringAnimation::SpringAnimation(SpringForce &&spring_force)
    : spring_(spring_force),
      pending_position_(UNSET),
      value_(UNSET),
      min_value(-UNSET),
      max_value(UNSET) {}

bool SpringAnimation::update_value_and_velocity(long delta_t) {
    running_ = true;

    if (end_requested_) {
        if (skip_requested_) {
            spring_.final_position = value_;
            pending_position_ = UNSET;
        } else if (pending_position_ != UNSET) {
            spring_.final_position = pending_position_;
            pending_position_ = UNSET;
        }
        value_ = static_cast<float>(spring_.final_position);
        velocity_ = 0;

        end_requested_ = false;
        skip_requested_ = false;
        return true;
    }

    if (pending_position_ != UNSET) {
        // Approximate by considering half of the time spring position stayed at the old
        // position, half of the time it's at the new position.
        auto mass_state = spring_.update_values(
            value_,
            velocity_,
            delta_t / 2
        );
        spring_.final_position = pending_position_;
        pending_position_ = UNSET;

        mass_state = spring_.update_values(
            mass_state.value,
            mass_state.velocity,
            delta_t / 2
        );
        value_ = mass_state.value;
        velocity_ = mass_state.velocity;
    } else {
        const auto mass_state = spring_.update_values(
            value_,
            velocity_,
            delta_t
        );
        value_ = mass_state.value;
        velocity_ = mass_state.velocity;
    }

    value_ = std::max(value_, min_value);
    value_ = std::min(value_, max_value);

    if (spring_.is_at_equilibrium(value_, velocity_)) {
        value_ = static_cast<float>(spring_.final_position);
        velocity_ = 0;
        running_ = false;
        return true;
    }
    return false;
}

void SpringAnimation::update_final_position(float final_position) {
    if (running_) {
        pending_position_ = final_position;
    } else {
        if (pending_position_ != UNSET) {
            pending_position_ = final_position;
        } else {
            spring_.final_position = final_position;
        }
    }
}