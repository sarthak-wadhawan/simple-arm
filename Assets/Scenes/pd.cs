using UnityEngine;

public class PDController
{
    public float Kp = 1.0f;
    public float Kd = 0.1f;
    private float previousError = 0.0f;

    public float Compute(float targetAngle, float currentAngle, float deltaTime)
    {
        float error = targetAngle - currentAngle;
        float derivative = (error - previousError) / deltaTime;
        previousError = error;
        return Kp * error + Kd * derivative;
    }
}
