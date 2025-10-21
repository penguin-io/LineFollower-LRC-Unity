using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Publishes IR sensor array data to ROS 2
/// Attach to ALFR robot GameObject
/// </summary>
public class IRArrayPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/alfr/ir_array";
    public float publishFrequency = 20.0f; // 20 Hz
    
    [Header("IR Sensor Configuration")]
    public int numSensors = 8;
    public float sensorSpacing = 0.01f; // 10mm between sensors
    public LayerMask lineLayer;
    
    private float timeElapsed;
    private IRsensorArray sensorArray; // Reference to existing sensor component
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Int32MultiArrayMsg>(topicName);
        
        // Get existing sensor component
        sensorArray = GetComponent<IRsensorArray>();
        if (sensorArray == null)
        {
            Debug.LogError("IRsensorArray component not found!");
        }
    }
    
    void Update()
    {
        timeElapsed += Time.deltaTime;
        
        if (timeElapsed > 1.0f / publishFrequency)
        {
            PublishIRData();
            timeElapsed = 0;
        }
    }
    
    void PublishIRData()
    {
        if (sensorArray == null) return;
        
        Int32MultiArrayMsg msg = new Int32MultiArrayMsg();
        
        // Get sensor readings from IRsensorArray component
        // Assuming IRsensorArray has a method to get raw values
        int[] rawValues = new int[numSensors];
        for (int i = 0; i < numSensors; i++)
        {
            // Get sensor value - adapt based on your IRsensorArray implementation
            rawValues[i] = sensorArray.GetSensorValue(i);
        }
        
        msg.data = rawValues;
        
        ros.Publish(topicName, msg);
    }
}
