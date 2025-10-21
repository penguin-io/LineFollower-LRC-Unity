using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Subscribes to gripper open/close commands
/// 0.0 = closed, 1.0 = open
/// </summary>
public class GripperSubscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/sarm/gripper/command";
    
    [Header("Gripper Configuration")]
    public ArticulationBody leftFinger;
    public ArticulationBody rightFinger;
    public float closedPosition = 0.0f;
    public float openPosition = 0.04f; // 4 cm separation
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float32Msg>(topicName, SetGripperPosition);
    }
    
    void SetGripperPosition(Float32Msg msg)
    {
        float command = msg.data; // 0-1 normalized
        float targetPos = Mathf.Lerp(closedPosition, openPosition, command);
        
        // Apply symmetric positions to both fingers
        if (leftFinger != null)
        {
            var drive = leftFinger.xDrive;
            drive.target = targetPos * Mathf.Rad2Deg;
            leftFinger.xDrive = drive;
        }
        
        if (rightFinger != null)
        {
            var drive = rightFinger.xDrive;
            drive.target = -targetPos * Mathf.Rad2Deg; // Mirror for right finger
            rightFinger.xDrive = drive;
        }
    }
}
