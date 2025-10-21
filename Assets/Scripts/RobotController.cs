using UnityEngine;

[RequireComponent(typeof(IRsensorArray))]
[RequireComponent(typeof(Rigidbody))]
public class RobotController : MonoBehaviour
{
    // --- Public Fields ---
    [Header("Wheel Configuration")]
    [Tooltip("The WheelCollider for the left wheel.")]
    public WheelCollider wheelLeft;
    [Tooltip("The WheelCollider for the right wheel.")]
    public WheelCollider wheelRight;
    [Tooltip("The Transform of the 3D model for the left wheel.")]
    public Transform wheelLeftTransform;
    [Tooltip("The Transform of the 3D model for the right wheel.")]
    public Transform wheelRightTransform;

    [Header("Speed Control (PID)")]
    [Tooltip("The target RPM for the wheels when driving straight.")]
    public float targetRPM = 100f;
    [Tooltip("The maximum torque the PID controller can apply to the wheels.")]
    public float maxMotorTorque = 1000f;
    [Tooltip("Proportional gain: How strongly the controller reacts to the current error.")]
    public float Kp_speed = 0.5f;
    [Tooltip("Integral gain: Corrects for past errors, helps eliminate steady-state error.")]
    public float Ki_speed = 0.1f;
    [Tooltip("Derivative gain: Predicts future error, helps dampen oscillations.")]
    public float Kd_speed = 0.01f;

    [Header("Steering Control")]
    [Tooltip("How sensitive the robot is to line errors for steering. This adjusts the RPM difference between wheels.")]
    public float steeringSensitivity = 50f;

    // --- Private PID State Variables ---
    private IRsensorArray sensorArray;
    private Rigidbody rb;

    // PID state for the left wheel
    private float integralLeft = 0f;
    private float lastErrorLeft = 0f;

    // PID state for the right wheel
    private float integralRight = 0f;
    private float lastErrorRight = 0f;

    void Start()
    {
        sensorArray = GetComponent<IRsensorArray>();
        rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.centerOfMass = new Vector3(0, -0.1f, 0);
        }
    }

    void FixedUpdate()
    {
        // 1. Get steering error from the sensor array (-1 for left, 1 for right)
        float lineError = sensorArray.lineError;

        // 2. Determine the target RPM for each wheel based on steering
        // If turning right (lineError > 0), the left wheel needs to speed up and the right wheel needs to slow down.
        float steeringAdjustment = lineError * steeringSensitivity;
        float targetRPM_Left = targetRPM + steeringAdjustment; // Was '-'
        float targetRPM_Right = targetRPM - steeringAdjustment; // Was '+'

        // 3. Get the current RPM of each wheel
        float currentRPM_Left = wheelLeft.rpm;
        float currentRPM_Right = wheelRight.rpm;

        // 4. Calculate the required torque for each wheel using its PID controller
        float torqueLeft = CalculatePIDOutput(targetRPM_Left, currentRPM_Left, ref integralLeft, ref lastErrorLeft);
        float torqueRight = CalculatePIDOutput(targetRPM_Right, currentRPM_Right, ref integralRight, ref lastErrorRight);

        // 5. Apply the calculated torque to the wheels
        wheelLeft.motorTorque = torqueLeft;
        wheelRight.motorTorque = torqueRight;
    }

    /// <summary>
    /// Calculates the necessary motor torque using a PID formula.
    /// </summary>
    private float CalculatePIDOutput(float target, float current, ref float integral, ref float lastError)
    {
        // Calculate the error
        float error = target - current;

        // --- Proportional Term ---
        float P = Kp_speed * error;

        // --- Integral Term ---
        // Accumulate error over time
        integral += error * Time.fixedDeltaTime;
        float I = Ki_speed * integral;

        // --- Derivative Term ---
        // Calculate the rate of change of the error
        float derivative = (error - lastError) / Time.fixedDeltaTime;
        float D = Kd_speed * derivative;
        lastError = error; // Update lastError for the next frame

        // --- Total Output ---
        // Sum the terms to get the final output torque
        float output = P + I + D;

        // Clamp the output to the maximum torque to prevent excessive force
        return Mathf.Clamp(output, -maxMotorTorque, maxMotorTorque);
    }

    void LateUpdate()
    {
        // Update the visual representation of the wheels
        UpdateWheelVisual(wheelLeft, wheelLeftTransform);
        UpdateWheelVisual(wheelRight, wheelRightTransform);
    }

    void UpdateWheelVisual(WheelCollider col, Transform tr)
    {
        if (tr == null) return;
        Vector3 position;
        Quaternion rotation;
        col.GetWorldPose(out position, out rotation);
        tr.position = position;
        tr.rotation = rotation;
    }
}
