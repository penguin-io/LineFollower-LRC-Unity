using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Publishes gate crossing events when robot passes trigger zones
/// Attach to Gate1, Gate2, Gate3 GameObjects as trigger colliders
/// </summary>
public class GateTriggerPublisher : MonoBehaviour
{
    ROSConnection ros;
    
    [Header("Gate Configuration")]
    public int gateNumber = 1; // 1, 2, or 3
    public string robotTag = "ALFR"; // Only trigger for ALFR
    
    private string topicName;
    private bool hasTriggered = false;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        topicName = $"/gates/g{gateNumber}_crossed";
        ros.RegisterPublisher<BoolMsg>(topicName);
        
        // Ensure this GameObject has a trigger collider
        Collider col = GetComponent<Collider>();
        if (col != null)
        {
            col.isTrigger = true;
        }
        else
        {
            Debug.LogWarning($"Gate {gateNumber} has no Collider component!");
        }
    }
    
    void OnTriggerEnter(Collider other)
    {
        // Only trigger once and only for ALFR robot
        if (!hasTriggered && other.CompareTag(robotTag))
        {
            PublishCrossingEvent(true);
            hasTriggered = true;
            Debug.Log($"Gate {gateNumber} crossed by {other.name}");
        }
    }
    
    void PublishCrossingEvent(bool crossed)
    {
        BoolMsg msg = new BoolMsg { data = crossed };
        ros.Publish(topicName, msg);
    }
    
    // Optional: Reset for testing
    public void ResetGate()
    {
        hasTriggered = false;
        PublishCrossingEvent(false);
        Debug.Log($"Gate {gateNumber} reset");
    }
}
