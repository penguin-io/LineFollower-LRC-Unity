using UnityEngine;

[RequireComponent(typeof(IRsensorArray))]
[RequireComponent(typeof(Rigidbody))]
public class RobotController : MonoBehaviour
{
    public WheelCollider wheelLeft;
    public WheelCollider wheelRight;
    public Transform wheelLeftTransform;
    public Transform wheelRightTransform;

    public float targetRPM = 100f;
    public float maxMotorTorque = 1000f;
    public float Kp_speed = 0.5f;
    public float Ki_speed = 0.1f;
    public float Kd_speed = 0.01f;

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
        float lineError = sensorArray.lineError;

        float steeringAdjustment = lineError * steeringSensitivity;
        float targetRPM_Left = targetRPM + steeringAdjustment; 
        float targetRPM_Right = targetRPM - steeringAdjustment;


        float currentRPM_Left = wheelLeft.rpm;
        float currentRPM_Right = wheelRight.rpm;

        float torqueLeft = CalculatePIDOutput(targetRPM_Left, currentRPM_Left, ref integralLeft, ref lastErrorLeft);
        float torqueRight = CalculatePIDOutput(targetRPM_Right, currentRPM_Right, ref integralRight, ref lastErrorRight);

        wheelLeft.motorTorque = torqueLeft;
        wheelRight.motorTorque = torqueRight;
    }

    private float CalculatePIDOutput(float target, float current, ref float integral, ref float lastError)
    {
        float error = target - current;
        float P = Kp_speed * error;

        integral += error * Time.fixedDeltaTime;
        float I = Ki_speed * integral;

        float derivative = (error - lastError) / Time.fixedDeltaTime;
        float D = Kd_speed * derivative;
        lastError = error;

        float output = P + I + D;

        return Mathf.Clamp(output, -maxMotorTorque, maxMotorTorque);
    }

    void LateUpdate()
    {
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
