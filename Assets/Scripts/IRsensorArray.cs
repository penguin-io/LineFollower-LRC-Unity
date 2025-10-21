using UnityEngine;

public class IRsensorArray : MonoBehaviour
{
    // --- Public Fields ---
    [Tooltip("Assign 5 sensor GameObjects in the Inspector, from left to right.")]
    public Transform[] sensorPositions;

    [Tooltip("How far down the sensors should check for the line.")]
    public float sensorLength = 1.0f;

    // Public variable for the RobotController to read.
    // [HideInInspector] makes it not show up in the Inspector but be public.
    [HideInInspector]
    public float lineError;

    // --- Private Fields ---
    // Stores the state of each sensor (true = sees line, false = off line).
    private bool[] sensorStates;

    void Start()
    {
        // Ensure the array is correctly sized
        if (sensorPositions.Length != 5)
        {
            Debug.LogError("This script requires exactly 5 sensors assigned in the sensorPositions array.");
            // Disable the component to prevent errors
            this.enabled = false;
            return;
        }

        // Initialize the sensor states array
        sensorStates = new bool[sensorPositions.Length];
        lineError = 0f;
    }

    void FixedUpdate()
    {
        // 1. Check the line under each sensor
        UpdateSensorStates();

        // 2. Calculate the error based on which sensors are active
        CalculateLineError();
    }

    /// <summary>
    /// Casts a ray down from each sensor's position to detect the line.
    /// The line must have the tag "Line".
    /// </summary>
    private void UpdateSensorStates()
    {
        for (int i = 0; i < sensorPositions.Length; i++)
        {
            RaycastHit hit;
            // Raycast downwards from the sensor's position
            if (Physics.Raycast(sensorPositions[i].position, -transform.up, out hit, sensorLength))
            {
                // If the ray hits an object tagged "Line", set the state to true
                sensorStates[i] = hit.collider.CompareTag("Line");
            }
            else
            {
                // Nothing was hit within the sensor's range
                sensorStates[i] = false;
            }
        }
    }

    /// <summary>
    /// Calculates a weighted error value based on the 5 sensor readings.
    /// This provides a more granular control signal than the 3-sensor setup.
    /// </summary>
    private void CalculateLineError()
    {
        // Sensor states from left to right [FarLeft, Left, Middle, Right, FarRight]
        bool s1 = sensorStates[0];
        bool s2 = sensorStates[1];
        bool s3 = sensorStates[2];
        bool s4 = sensorStates[3];
        bool s5 = sensorStates[4];

        // --- Line is centered ---
        // Pattern: 00100
        if (!s1 && !s2 && s3 && !s4 && !s5)
        {
            lineError = 0f; // Perfect on line, go straight
        }

        // --- Robot is slightly to the right of the line (needs to turn left) ---
        // Pattern: 01100
        else if (!s1 && s2 && s3 && !s4 && !s5)
        {
            lineError = -0.25f; // Gentle left
        }
        // Pattern: 01000
        else if (!s1 && s2 && !s3 && !s4 && !s5)
        {
            lineError = -0.5f; // Medium left
        }

        // --- Robot is far to the right of the line ---
        // Pattern: 11000
        else if (s1 && s2 && !s3 && !s4 && !s5)
        {
            lineError = -0.75f; // Strong left
        }
        // Pattern: 10000
        else if (s1 && !s2 && !s3 && !s4 && !s5)
        {
            lineError = -1.0f; // Hard left
        }

        // --- Robot is slightly to the left of the line (needs to turn right) ---
        // Pattern: 00110
        else if (!s1 && !s2 && s3 && s4 && !s5)
        {
            lineError = 0.25f; // Gentle right
        }
        // Pattern: 00010
        else if (!s1 && !s2 && !s3 && s4 && !s5)
        {
            lineError = 0.5f; // Medium right
        }

        // --- Robot is far to the left of the line ---
        // Pattern: 00011
        else if (!s1 && !s2 && !s3 && s4 && s5)
        {
            lineError = 0.75f; // Strong right
        }
        // Pattern: 00001
        else if (!s1 && !s2 && !s3 && !s4 && s5)
        {
            lineError = 1.0f; // Hard right
        }

        // --- Special Cases ---
        // All sensors on the line (intersection or start line)
        else if (s1 && s2 && s3 && s4 && s5)
        {
            lineError = 0f; // Go straight
        }
        // Line is lost (all sensors off)
        else if (!s1 && !s2 && !s3 && !s4 && !s5)
        {
            // If the line is lost, continue with the last known action.
            // For example, if it was turning hard right (error = 1.0), it will continue
            // turning right, which helps it find the line again on a sharp curve.
            // No change to lineError is needed here.
        }
    }

    /// <summary>
    /// Draws visual lines in the Scene view to show sensor rays.
    /// Green = Line Detected, Red = No Line.
    /// </summary>
    void OnDrawGizmos()
    {
        if (sensorPositions == null || sensorStates == null || sensorPositions.Length != sensorStates.Length) return;

        for (int i = 0; i < sensorPositions.Length; i++)
        {
            Vector3 rayStart = sensorPositions[i].position;
            Vector3 rayEnd = rayStart - transform.up * sensorLength;

            // Set gizmo color based on sensor state
            Gizmos.color = sensorStates[i] ? Color.green : Color.red;
            Gizmos.DrawLine(rayStart, rayEnd);
        }
    }
}
