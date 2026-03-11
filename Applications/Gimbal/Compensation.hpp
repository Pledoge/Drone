#pragma once
#include "AppConfig.h"

namespace Compensation
{

#if USE_DEBUG

static volatile float stable_pitch_angle[100];
static volatile float stable_yaw_angle[100];

static volatile float stable_pitch_output[100];
static volatile float stable_yaw_output[100];

static volatile uint16_t stable_pitch_index = 0;
static volatile uint16_t stable_yaw_index   = 0;

template <typename T>
class MovingAverageFilter
{
   public:
    MovingAverageFilter() : head_(0), count_(0), sum_(0)
    {
        for (size_t i = 0; i < WINDOW_SIZE; i++)
        {
            buffer_[i] = T(0);
        }
    }

    T update(T input)
    {
        if (count_ >= WINDOW_SIZE)
        {
            sum_ -= buffer_[head_];
        }
        else
        {
            count_++;
        }

        buffer_[head_] = input;
        sum_ += input;

        head_ = (head_ + 1) % WINDOW_SIZE;

        return sum_ / count_;
    }

    void reset()
    {
        head_  = 0;
        count_ = 0;
        sum_   = 0;
        for (size_t i = 0; i < WINDOW_SIZE; i++)
        {
            buffer_[i] = T(0);
        }
    }

    bool isStable() const { return count_ >= WINDOW_SIZE; }

   private:
    static constexpr size_t WINDOW_SIZE = 400;
    T buffer_[WINDOW_SIZE];
    size_t head_;
    size_t count_;
    T sum_;
};

MovingAverageFilter<float> pitch_filter[2];
MovingAverageFilter<float> yaw_filter[2];

bool updatePitchCompensationData(float angle, float output)
{
    if (pitch_filter[0].isStable() && pitch_filter[1].isStable())
    {
        stable_pitch_index += 1;
        stable_pitch_angle[stable_pitch_index]  = angle;
        stable_pitch_output[stable_pitch_index] = output;
        pitch_filter[0].reset();
        pitch_filter[1].reset();
        return true;
    }
    pitch_filter[0].update(angle);
    pitch_filter[1].update(output);
    return false;
}

bool updateYawCompensationData(float angle, float output)
{
    if (yaw_filter[0].isStable() && yaw_filter[1].isStable())
    {
        stable_yaw_index += 1;
        stable_yaw_angle[stable_yaw_index]  = angle;
        stable_yaw_output[stable_yaw_index] = output;
        yaw_filter[0].reset();
        yaw_filter[1].reset();
        return true;
    }
    yaw_filter[0].update(angle);
    yaw_filter[1].update(output);
    return false;
}

// Pitch Gravity compensation method:
// #define ONE_STEP_ANGLE -0.01f
//             static bool finish_one_step = false;
//             static bool finish_all      = false;
//             static uint32_t start_time  = xTaskGetTickCount();
//             if (!finish_all)
//             {
//                 uint32_t current_time = xTaskGetTickCount();

//                 if (current_time - start_time > pdMS_TO_TICKS(5000))
//                 {
//                     finish_one_step =
//                         Compensation::updatePitchCompensationData(gimbal_controller.getCurrentGimbalEuler().pitch,
//                         pitch_motor.getCurrentFeedback());
//                 }
//                 if (finish_one_step)
//                 {
//                     gimbal_controller.changeTargetPosByCommand(0, ONE_STEP_ANGLE);
//                     start_time      = xTaskGetTickCount();
//                     finish_one_step = false;
//                 }
//                 if (gimbal_controller.getTargetMotorEuler().pitch + ONE_STEP_ANGLE <= MIN_PITCH_ANGLE)
//                 {
//                     finish_all = true;
//                     gimbal_controller.setCompensating(false);
//                 }
//             }
//             gimbal_controller.updateTargetGimbalPos(0, 0);
//             gimbal_controller.setPitchMotorOutput();
//             gimbal_controller.setYawMotorOutput();
//             break;

// #undef ONE_STEP_ANGLE

// #define FRICTION_COMPENSATION_SPEED 20.0f
//             static volatile float positive_output = 0.0f;
//             static volatile float negative_output = 0.0f;
//             static volatile float positive_speed  = 0.0f;
//             static volatile float negative_speed  = 0.0f;
//             static volatile float current_speed   = 0.0f;
//             static Core::Control::PID::Param compensation_pid_param(90, 0, 0.001f, 100000, 100000, 0);
//             static Core::Control::PID compensation_pid(compensation_pid_param);
//             static int finish_count     = 0;
//             static bool finish_one_step = false;
//             if (finish_count == 60)
//             {
//                 gimbal_controller.setCompensating(false);
//             }
//             if (yaw_motor.getPositionFeedback() > 0.50f && !finish_one_step)
//             {
//                 finish_one_step = true;
//                 finish_count++;
//                 current_speed   = -FRICTION_COMPENSATION_SPEED * (finish_count / 20 + 1);
//                 positive_output = 0.0f;
//                 positive_speed  = 0.0f;
//                 negative_output = 0.0f;
//                 negative_speed  = 0.0f;
//             }
//             else if (yaw_motor.getPositionFeedback() < -0.50f && !finish_one_step)
//             {
//                 finish_one_step = true;
//                 finish_count++;
//                 current_speed   = FRICTION_COMPENSATION_SPEED * (finish_count / 20 + 1);
//                 positive_output = 0.0f;
//                 positive_speed  = 0.0f;
//                 negative_output = 0.0f;
//                 negative_speed  = 0.0f;
//             }
//             else if (yaw_motor.getPositionFeedback() > -0.40f && yaw_motor.getPositionFeedback() < 0.40f)
//             {
//                 finish_one_step = false;
//                 if (current_speed > 0)
//                 {
//                     positive_output = yaw_motor.getOutput();
//                     positive_speed  = yaw_motor.getRPMFeedback();
//                     negative_output = 0.0f;
//                     negative_speed  = 0.0f;
//                 }
//                 else if (current_speed < 0)
//                 {
//                     negative_output = yaw_motor.getOutput();
//                     negative_speed  = yaw_motor.getRPMFeedback();
//                     positive_output = 0.0f;
//                     positive_speed  = 0.0f;
//                 }
//             }
//             pitch_motor.setCurrent(0);
//             yaw_motor.setOutput(compensation_pid(current_speed, yaw_motor.getRPMFeedback()));

#endif
}  // namespace Compensation