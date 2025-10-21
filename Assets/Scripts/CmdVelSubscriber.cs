using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

/// <summary>
/// Subscribes to /sarm/cmd_vel and drives mecanum/omni wheels
/// Attach to SARM robot GameObject
/// </summary>
public class CmdVelSubscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/sarm/cmd_vel";
    
    [Header("Mecanum Wheel Configuration")]
    public float wheelRadius = 0.05f; // 5 cm
    public float robotLength = 0.3f;  // L in kinematics
    public float robotWidth = 0.25f;  // W in kinematics
    
    // References to wheel ArticulationBodies
    public ArticulationBody wheelFrontLeft;
    public ArticulationBody wheelFrontRight;
    public ArticulationBody wheelBackLeft;
    public ArticulationBody wheelBackRight;
    
    private Vector3 targetVelocity; // vx, vy, omega_z
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, ApplyVelocity);
    }
    
    void ApplyVelocity(TwistMsg twist)
    {
        // Extract velocities from Twist message
        float vx = (float)twist.linear.x;
        float vy = (float)twist.linear.y;
        float omega = (float)twist.angular.z;
        
        // Mecanum forward kinematics
        float lPlusW = robotLength + robotWidth;
        
        // Wheel angular velocities (rad/s) from body twist
        float w1 = (vx - vy - lPlusW * omega) / wheelRadius; // Front-left
        float w2 = (vx + vy + lPlusW * omega) / wheelRadius; // Front-right
        float w3 = (vx + vy - lPlusW * omega) / wheelRadius; // Back-left
        float w4 = (vx - vy + lPlusW * omega) / wheelRadius; // Back-right
        
        // Apply to ArticulationBody drives
        SetWheelVelocity(wheelFrontLeft, w1);
        SetWheelVelocity(wheelFrontRight, w2);
        SetWheelVelocity(wheelBackLeft, w3);
        SetWheelVelocity(wheelBackRight, w4);
    }
    
    void SetWheelVelocity(ArticulationBody wheel, float angularVel)
    {
        if (wheel == null) return;
        
        var drive = wheel.xDrive;
        drive.targetVelocity = angularVel * Mathf.Rad2Deg; // Unity uses degrees/sec
        wheel.xDrive = drive;
    }
}
