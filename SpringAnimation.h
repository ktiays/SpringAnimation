//
// Created by ktiays on 2023/1/18.
// Copyright (c) 2023 ktiays. All rights reserved.
//

#ifndef SPRINGKIT_SPRINGANIMATION_H
#define SPRINGKIT_SPRINGANIMATION_H

#include "SpringForce.h"

/// @c SpringAnimation is an animation that is driven by a @c SpringForce. The spring force defines
/// the spring's stiffness, damping ratio, as well as the rest position. Once the @c SpringAnimation is
/// started, on each frame the spring force will update the animation's value and velocity.
/// The animation will continue to run until the spring force reaches equilibrium. If the spring used
/// in the animation is undamped, the animation will never reach equilibrium. Instead, it will
/// oscillate forever.
class SpringAnimation {
private:
    SpringForce spring_;
    float pending_position_;

    // A Boolean value indicates whether to skip the uncompleted part of the animation when the
    // animation is manually terminated.
    bool skip_requested_ = false;
    bool end_requested_ = false;

    float velocity_ = 0;
    float value_;

    /// A Boolean value indicates whether animation is running.
    bool running_ = false;

public:
    float min_value;
    float max_value;

    SpringAnimation();

    SpringAnimation(float final_position);

    SpringAnimation(const SpringForce &spring_force);

    SpringAnimation(SpringForce &&spring_force);

    bool update_value_and_velocity(long delta_t);

    /// Updates the final position of the spring.
    /// <p/>
    /// When the animation is running, calling this method would assume the position change of the
    /// spring as a continuous movement since last frame, which yields more accurate results than
    /// changing the spring position directly.
    /// <p/>
    /// If the animation hasn't started, calling this method will change the spring position, and
    /// immediately start the animation
    ///
    /// @param final_position rest position of the spring
    void update_final_position(float final_position);

    /// Start velocity of the animation. Default velocity is 0. Unit: change in property per
    /// second (e.g. pixels per second, scale/alpha value change per second).
    inline SpringAnimation &set_start_velocity(float start_velocity) {
        velocity_ = start_velocity;
        return *this;
    }

    /// Sets the start value of the animation.
    inline SpringAnimation &set_start_value(float value) {
        value_ = value;
        return *this;
    }

    inline float value() const {
        return value_;
    }

    /// Skips to the end of the animation.
    inline void skip_to_end() {
        end_requested_ = true;
        skip_requested_ = true;
    }

    /// Cancels the on-going animation.
    /// <p>
    /// The animation will stop at the current state.
    /// </p>
    inline void interrupt() {
        end_requested_ = true;
        skip_requested_ = false;
    }

};

#endif //SPRINGKIT_SPRINGANIMATION_H
