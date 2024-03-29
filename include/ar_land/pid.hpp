#pragma once

#include <ros/ros.h>

class PID
{
public:
    PID(
        float kp,
        float kd,
        float ki,
        float minOutput,
        float maxOutput,
        float integratorMin,
        float integratorMax,
        const std::string& name)
        : m_kp(kp)
        , m_kd(kd)
        , m_ki(ki)
        , m_minOutput(minOutput)
        , m_maxOutput(maxOutput)
        , m_integratorMin(integratorMin)
        , m_integratorMax(integratorMax)
        , m_integral(0)
        , m_previousError(0)
        , m_previousTime(ros::Time::now())
    {
    }

    void reset()
    {
        m_integral = 0;
        m_previousError = 0;
        m_previousTime = ros::Time::now();
    }

    void set(float integral)
    {
        m_integral = integral;
        m_previousError = 0;
        m_previousTime = ros::Time::now();
    }

    void setIntegral(float integral)
    {
        m_integral = integral;
    }

    float getI()
    {
      return m_ki * m_integral;
    }

    float getD()
    {
      return d;
    }

    float getP()
    {
      return p;
    }
    float getError()
    {
      return error;
    }

    float ki() const
    {
      return m_ki;
    }

    void setKI(float ki)
    {
      m_ki = ki;
    }

    void setKD(float kd)
    {
      m_kd = kd;
    }

    void setKP(float kp)
    {
      m_kp = kp;
    }


    float getOutput()
    {
      float output = m_ki*m_integral + p + d;
     return  std::max(std::min(output, m_maxOutput), m_minOutput);
    }

    float update(float value, float targetValue)
    {
        ros::Time time = ros::Time::now();
        float dt = time.toSec() - m_previousTime.toSec();
        error = targetValue - value;
        m_integral += error * dt;
        m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
        p = m_kp * error;
        d = 0;
        if (dt > 0)
        {
            d = m_kd * (error - m_previousError) / dt;
        }
        float i = m_ki * m_integral;
        float output = p + d + i;
        m_previousError = error;
        m_previousTime = time;
        // self.pubOutput.publish(output)
        // self.pubError.publish(error)
        // self.pubP.publish(p)
        // self.pubD.publish(d)
        // self.pubI.publish(i)
        return std::max(std::min(output, m_maxOutput), m_minOutput);
    }

private:
    float m_kp;
    float m_kd;
    float m_ki;
    float m_minOutput;
    float m_maxOutput;
    float m_integratorMin;
    float m_integratorMax;
    float m_integral;
    float m_previousError;
    float d;
    float p;
    float error;
    ros::Time m_previousTime;
};
