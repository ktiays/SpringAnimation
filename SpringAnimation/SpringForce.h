//
// Created by ktiays on 2023/1/18.
// Copyright (c) 2023 ktiays. All rights reserved.
//

#ifndef SPRINGKIT_SPRINGFORCE_H
#define SPRINGKIT_SPRINGFORCE_H

/// Spring Force defines the characteristics of the spring being used in the animation.
/// <p>
/// By configuring the stiffness and damping ratio, callers can create a spring with the look and
/// feel suits their use case. Stiffness corresponds to the spring constant. The stiffer the spring
/// is, the harder it is to stretch it, the faster it undergoes dampening.
/// </p>
/// Spring damping ratio describes how oscillations in a system decay after a disturbance.
/// When damping ratio > 1
/// (i.e. over-damped), the object will quickly return to the rest position
/// without overshooting. If damping ratio equals to 1 (i.e. critically damped), the object will
/// return to equilibrium within the shortest amount of time. When damping ratio is less than 1
/// (i.e. under-damped), the mass tends to overshoot, and return, and overshoot again. Without any
/// damping (i.e. damping ratio = 0), the mass will oscillate forever.
class SpringForce {
private:
    struct MassState final {
        float value;
        float velocity;
    };

    // Threshold for velocity and value to determine when it's reasonable to assume that the spring
    // is approximately at rest.
    double value_threshold_ = 0;
    double velocity_threshold_ = 0;

    // Intermediate values to simplify the spring function calculation per frame.
    double gamma_plus_ = 0;
    double gamma_minus_ = 0;
    double damped_freq_ = 0;

    // This multiplier is used to calculate the velocity threshold given a certain value threshold.
    // The idea is that if it takes >= 1 frame to move the value threshold amount, then the velocity
    // is a reasonable threshold.
    double velocity_threshold_multiplier_ = 1000.f / 16.f;

    // Natural frequency
    double natural_freq_;
    // Damping ratio.
    double damping_ratio_;

    bool initialized_ = false;
    MassState state_;

    float acceleration(float last_displacement, float last_velocity) const;

    bool is_at_equilibrium(float value, float velocity) const;

    void initialize();

    /// Internal only call for Spring to calculate the spring position/velocity using
    /// an analytical approach.
    MassState update_values(double last_displacement, double last_velocity, long time_elapsed);

    friend class SpringAnimation;

public:
    struct StiffnessConstant final {
        /// Stiffness constant for extremely stiff spring.
        static float high;
        /// Stiffness constant for medium stiff spring. This is the default stiffness for spring force.
        static float medium;
        /// Stiffness constant for a spring with low stiffness.
        static float low;
        /// Stiffness constant for a spring with very low stiffness.
        static float very_low;
    };

    struct DampingRatioConstant final {
        /// Damping ratio for a very bouncy spring. Note for under-damped springs
        /// (i.e. damping ratio < 1), the lower the damping ratio, the more bouncy the spring.
        static float high_bouncy;
        /// Damping ratio for a medium bouncy spring. This is also the default damping ratio for spring
        /// force. Note for under-damped springs (i.e. damping ratio < 1), the lower the damping ratio,
        /// the more bouncy the spring.
        static float medium_bouncy;
        /// Damping ratio for a spring with low bounciness. Note for under-damped springs
        /// (i.e. damping ratio < 1), the lower the damping ratio, the higher the bounciness.
        static float low_bouncy;
        /// Damping ratio for a spring with no bounciness. This damping ratio will create a critically
        /// damped spring that returns to equilibrium within the shortest amount of time without
        /// oscillating.
        static float no_bouncy;
    };

    double final_position;

    /// Creates a spring force. Note that final position of the spring must be set before the
    /// spring animation starts.
    SpringForce();

    /// Creates a spring with a given final rest position.
    ///
    /// @param final_position final position of the spring when it reaches equilibrium.
    SpringForce(float final_position);

    /// Sets the stiffness of a spring. The more stiff a spring is, the more force it applies to
    /// the object attached when the spring is not at the final position. Default stiffness is
    /// @c StiffnessConstant::medium.
    ///
    /// @param stiffness non-negative stiffness constant of a spring.
    /// @return the spring force that the given stiffness is set on.
    SpringForce &set_stiffness(float stiffness);

    /// Gets the stiffness of the spring.
    inline float stiffness() const {
        return static_cast<float>(natural_freq_ * natural_freq_);
    }

    /// Spring damping ratio describes how oscillations in a system decay after a disturbance.
    /// <p>
    /// When damping ratio > 1 (over-damped), the object will quickly return to the rest position
    /// without overshooting. If damping ratio equals to 1 (i.e. critically damped), the object will
    /// return to equilibrium within the shortest amount of time. When damping ratio is less than 1
    /// (i.e. under-damped), the mass tends to overshoot, and return, and overshoot again. Without
    /// any damping (i.e. damping ratio = 0), the mass will oscillate forever.
    /// </p>
    /// Default damping ratio is @c DampingRatioConstant::medium_bouncy.
    ///
    /// @param damping_ratio damping ratio of the spring, it should be non-negative.
    /// @return the spring force that the given damping ratio is set on.
    SpringForce &set_damping_ratio(float damping_ratio);

    /// Returns the damping ratio of the spring.
    inline float damping_ratio() const {
        return static_cast<float>(damping_ratio_);
    }

    SpringForce &set_frame_rate(float frame_rate) {
        velocity_threshold_multiplier_ = frame_rate;
        return *this;
    }

    /// This threshold defines how close the animation value needs to be before the animation can
    /// finish. This default value is based on the property being animated, e.g. animations on alpha,
    /// scale, translation or rotation would have different thresholds. This value should be small
    /// enough to avoid visual glitch of "jumping to the end". But it shouldn't be so small that
    /// animations take seconds to finish.
    ///
    /// @param threshold the difference between the animation value and final spring position that
    ///                  is allowed to end the animation when velocity is very low
    SpringForce &set_value_threshold(double threshold);

};

#endif //SPRINGKIT_SPRINGFORCE_H
