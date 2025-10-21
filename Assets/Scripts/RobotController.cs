using UnityEngine;

[RequireComponent(typeof(IRsensorArray))] // Ensures the sensor script is on the same object
[RequireComponent(typeof(Rigidbody))]      // Ensures the robot has a Rigidbody
public class RobotController : MonoBehaviour
{
    // --- Public Fields ---
    [Header("Motor Control")]
    public WheelCollider wheelLeft;
    public WheelCollider wheelRight;

    [Header("Movement Parameters")]
    public float baseSpeed = 150f; // Base torque applied to both wheels
    public float Kp = 100f;        // Proportional gain (how strongly it reacts to error)

    // --- Private Fields ---
    private IRsensorArray sensorArray; // Reference to our sensor script
    private Rigidbody rb;

    void Start()
    {
        // Get the other components attached to this GameObject
        sensorArray = GetComponent<IRsensorArray>();
        rb = GetComponent<Rigidbody>();

        // Set a low center of mass to prevent the robot from flipping
        if (rb != null)
        {
            rb.centerOfMass = new Vector3(0, -0.1f, 0);
        }
    }

    void FixedUpdate()
    {
        // 1. Get the error signal from the sensor script
        // This value is calculated in IRSensorArray.cs
        float error = sensorArray.lineError;

        // 2. Calculate the correction force (P-Controller)
        // This is the "brain" of the line follower
        // Kp * error determines how much to turn
        float correction = Kp * error;

        // 3. Calculate motor torque for each wheel
        // If error > 0 (robot is left), turn right: (Left wheel faster, Right wheel slower)
        // If error < 0 (robot is right), turn left: (Left wheel slower, Right wheel faster)
        float leftTorque = baseSpeed - correction;
        float rightTorque = baseSpeed + correction;

        // 4. Apply torque to the wheels
        wheelLeft.motorTorque = leftTorque;
        wheelRight.motorTorque = rightTorque;
    }
}