using UnityEngine;

public class SARcontroller : MonoBehaviour
{
    public Rigidbody platformRigidbody;
    public float moveSpeed = 5f;

    public Transform swivelBase;

    public Transform upperArm;
    public Transform forearm;

    public Transform gripperBase;

    public float rotationSpeed = 50f;

    

    public Transform gripperLeft;
    
    public Transform gripperRight;
    
    public float gripperSpeed = 0.5f;
  
    public float minGripperGap = 0.1f;

    public float maxGripperGap = 1.0f;

    // --- Private Fields ---
    private Vector3 moveInput;

    void Update()
    {

        moveInput = new Vector3(Input.GetAxis("Horizontal"), 0f, Input.GetAxis("Vertical"));

        HandleArmMovement();
        HandleGripperMovement();
    }

    void FixedUpdate()
    {
        HandlePlatformMovement();
    }


    private void HandlePlatformMovement()
    {
        if (platformRigidbody == null) return;


        platformRigidbody.linearVelocity = moveInput * moveSpeed;
    }


    private void HandleArmMovement()
    {
        // Swivel Base (Q/E Keys)
        if (Input.GetKey(KeyCode.Q))
            swivelBase.Rotate(Vector3.up, -rotationSpeed * Time.deltaTime);
        if (Input.GetKey(KeyCode.E))
            swivelBase.Rotate(Vector3.up, rotationSpeed * Time.deltaTime);

        // Upper Arm (R/F Keys)
        if (Input.GetKey(KeyCode.R))
            upperArm.Rotate(Vector3.right, -rotationSpeed * Time.deltaTime);
        if (Input.GetKey(KeyCode.F))
            upperArm.Rotate(Vector3.right, rotationSpeed * Time.deltaTime);

        // Forearm (T/G Keys)
        if (Input.GetKey(KeyCode.T))
            forearm.Rotate(Vector3.right, -rotationSpeed * Time.deltaTime);
        if (Input.GetKey(KeyCode.G))
            forearm.Rotate(Vector3.right, rotationSpeed * Time.deltaTime);

        // Gripper Base (Y/H Keys)
        if (Input.GetKey(KeyCode.Y))
            gripperBase.Rotate(Vector3.right, -rotationSpeed * Time.deltaTime);
        if (Input.GetKey(KeyCode.H))
            gripperBase.Rotate(Vector3.right, rotationSpeed * Time.deltaTime);
    }


    private void HandleGripperMovement()
    {
        // Calculate the current distance between fingers
        float currentGap = Vector3.Distance(gripperLeft.position, gripperRight.position);

        // Close Gripper (Z Key)
        if (Input.GetKey(KeyCode.Z) && currentGap > minGripperGap)
        {
            gripperLeft.Translate(Vector3.right * gripperSpeed * Time.deltaTime);
            gripperRight.Translate(Vector3.left * gripperSpeed * Time.deltaTime);
        }

        // Open Gripper (X Key)
        if (Input.GetKey(KeyCode.X) && currentGap < maxGripperGap)
        {
            gripperLeft.Translate(Vector3.left * gripperSpeed * Time.deltaTime);
            gripperRight.Translate(Vector3.right * gripperSpeed * Time.deltaTime);
        }
    }
}

