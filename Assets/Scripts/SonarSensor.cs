using UnityEngine;

public class SonarSensor : MonoBehaviour
{
    public float detectionDistance = 5f;

    public LayerMask obstacleLayers;
    public bool isObstacleDetected;

    [SerializeField] private Transform sensorPos;

    void FixedUpdate()
    {
        RaycastHit hit;
        if (Physics.Raycast(sensorPos.position, transform.forward, out hit, detectionDistance, obstacleLayers))
        {

            isObstacleDetected = true;
        }
        else
        {

            isObstacleDetected = false;
        }
    }


    void OnDrawGizmos()
    {

        if (isObstacleDetected)
        {
            Gizmos.color = Color.red; 
        }
        else
        {
            Gizmos.color = Color.green; 
        }

        Gizmos.DrawLine(sensorPos.position, sensorPos.position + transform.forward * detectionDistance);
    }
}
