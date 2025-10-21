using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

/// <summary>
/// Publishes sonar/ultrasonic distance sensor data
/// Uses existing SonarSensor component
/// </summary>
public class SonarPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/alfr/sonar";
    public float publishFrequency = 10.0f; // 10 Hz
    
    [Header("Sonar Configuration")]
    public float minRange = 0.02f; // 2 cm
    public float maxRange = 4.0f;  // 400 cm
    public float fieldOfView = 15.0f; // degrees
    
    private float timeElapsed;
    private SonarSensor sonarSensor;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RangeMsg>(topicName);
        
        sonarSensor = GetComponent<SonarSensor>();
        if (sonarSensor == null)
        {
            Debug.LogError("SonarSensor component not found!");
        }
    }
    
    void Update()
    {
        timeElapsed += Time.deltaTime;
        
        if (timeElapsed > 1.0f / publishFrequency)
        {
            PublishRange();
            timeElapsed = 0;
        }
    }
    
    void PublishRange()
    {
        if (sonarSensor == null) return;
        
        RangeMsg msg = new RangeMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time - (int)Time.time) * 1e9)
                },
                frame_id = "alfr_sonar"
            },
            radiation_type = RangeMsg.ULTRASOUND,
            field_of_view = fieldOfView * Mathf.Deg2Rad,
            min_range = minRange,
            max_range = maxRange,
            range = sonarSensor.distance
        };
        
        ros.Publish(topicName, msg);
    }
}
