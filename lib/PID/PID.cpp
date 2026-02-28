#include "PID.h"
#include "Arduino.h"
#include <cmath>

PID::PID(double kp, double ki, double kd, double max_output) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_output_ = max_output;
    max_integral_ = 100.0;
    last_error_ = 0;    
    sum_error_ = 0;
    last_time_ = millis();
    last_output_ = 0;
}

double PID::clamp(double x, double lo, double hi){ //Limiting Hi and Lo
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

double PID::Calculate(double error)
{
    const double I_DEADBAND = 1.5; // Deadband for intehral in degrees

    unsigned long current_time = millis();

    if (current_time - last_time_ >= 20) // Minimun dt of 20 ms
    {
        double delta_time = (current_time - last_time_) / 1000.0;

        if (delta_time <= 0 || delta_time > 1.0) {
            last_time_ = current_time;
            last_error_ = error;
            return last_output_; //Returns last output if dt is invalid
        }

        //Propotional term
        double proportional = kp_ * error;

        //Integral term with trapezoidal rule and anti-windup
        double integral = 0;

        if (fabs(error) > I_DEADBAND) { //Checking if absolute value is greater than deadband
            
            double integral_increment = ((error + last_error_) / 2.0) * delta_time;//Possible increment using trapezoids
            double new_sum = sum_error_ + integral_increment; //Possbile new sum

            double test_output = proportional + (ki_ * new_sum);//checking possible output with new sum

            bool would_saturate = (fabs(test_output) > max_output_);//cheching if it would saturate with new sum
            bool error_reducing_saturation =
                ((test_output > 0 && error < 0) || (test_output < 0 && error > 0)); //Cheching if the error would reduce saturation

            if (!would_saturate || error_reducing_saturation) { //If theres no saturation or if the error would reduce saturation
                sum_error_ = new_sum;
                sum_error_ = clamp(sum_error_, -max_integral_, max_integral_);
            }
        }

        integral = ki_ * sum_error_;

        //Derivative term
        double delta_error = (error - last_error_) / delta_time;
        double derivative = kd_ * delta_error;

        // Final output
        double output = proportional + integral + derivative;
        output = clamp(output, -max_output_, max_output_);

        // Updatng state for next iteration
        last_error_ = error;
        last_time_ = current_time;
        last_output_ = output;

        return output;
    }
    return last_output_;
}

void PID::Reset()
{
    sum_error_ = 0;
    last_error_ = 0;
    last_time_ = millis();
    last_output_ = 0;
}

void PID::SetTunings(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::SetMaxOutput(double max_output)
{
    max_output_ = max_output;
}

void PID::SetMaxIntegral(double max_integral)
{
    max_integral_ = max_integral;
}

double PID::GetIntegral()
{
    return sum_error_;
}
