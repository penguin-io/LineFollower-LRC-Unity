using UnityEngine;

[RequireComponent(typeof(IRsensorArray))]
[RequireComponent(typeof(Rigidbody))]
public class RobotController : MonoBehaviour
{
    // --- Public Fields ---
    [Header("Sensor Configuration")]
    [Tooltip("Assign the Sonar Sensor GameObject here.")]
    public SonarSensor sonarSensor;

    [Header("Wheel Configuration")]
    public WheelCollider wheelLeft;
    public WheelCollider wheelRight;
    public Transform wheelLeftTransform;
    public Transform wheelRightTransform;

    [Header("Movement Control")]
    [Tooltip("The force applied to the brakes when an obstacle is detected.")]
    public float brakeForce = 5000f;
    public float targetRPM = 100f;
    public float maxMotorTorque = 1000f;
    public float steeringSensitivity = 50f;

    [Header("Speed PID Gains")]
    public float Kp_speed = 0.5f;
    public float Ki_speed = 0.1f;
    public float Kd_speed = 0.01f;


    private IRsensorArray sensorArray;
    private Rigidbody rb;
    private float integralLeft, lastErrorLeft;
    private float integralRight, lastErrorRight;

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

        if (sonarSensor != null && sonarSensor.isObstacleDetected)
        {
            ApplyBrakes();
            return;
        }

        ReleaseBrakes();
        HandleLineFollowing();
    }


    private void ApplyBrakes()
    {
        wheelLeft.motorTorque = 0;
        wheelRight.motorTorque = 0;
        wheelLeft.brakeTorque = brakeForce;
        wheelRight.brakeTorque = brakeForce;
    }


    private void ReleaseBrakes()
    {
        wheelLeft.brakeTorque = 0;
        wheelRight.brakeTorque = 0;
    }

    private void HandleLineFollowing()
    {
        float lineError = sensorArray.lineError;
        float steeringAdjustment = lineError * steeringSensitivity;
        float targetRPM_Left = targetRPM + steeringAdjustment;
        float targetRPM_Right = targetRPM - steeringAdjustment;

        float torqueLeft = CalculatePIDOutput(targetRPM_Left, wheelLeft.rpm, ref integralLeft, ref lastErrorLeft);
        float torqueRight = CalculatePIDOutput(targetRPM_Right, wheelRight.rpm, ref integralRight, ref lastErrorRight);

        wheelLeft.motorTorque = torqueLeft;
        wheelRight.motorTorque = torqueRight;
    }

    private float CalculatePIDOutput(float target, float current, ref float integral, ref float lastError)
    {
        float error = target - current;
        integral += error * Time.fixedDeltaTime;
        float derivative = (error - lastError) / Time.fixedDeltaTime;
        lastError = error;
        float output = (Kp_speed * error) + (Ki_speed * integral) + (Kd_speed * derivative);
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
        col.GetWorldPose(out Vector3 position, out Quaternion rotation);
        tr.position = position;
        tr.rotation = rotation;
    }
}