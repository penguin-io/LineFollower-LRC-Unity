using UnityEngine;
using System.Collections.Generic;

public class SARcontroller : MonoBehaviour
{
    [Header("Platform Movement")]
    [Tooltip("The Rigidbody component of the robot's base platform.")]
    public Rigidbody platformRigidbody;
    [Tooltip("The speed at which the platform moves on the XZ plane.")]
    public float moveSpeed = 5f;

    [Header("Arm Joints")]
    [Tooltip("The part of the base that swivels left and right.")]
    public Transform swivelBase;
    [Tooltip("The first link of the arm that rotates up and down.")]
    public Transform upperArm;
    [Tooltip("The second link of the arm that rotates up and down.")]
    public Transform forearm;
    [Tooltip("The wrist/base of the gripper that rotates up and down.")]
    public Transform gripperBase;
    [Tooltip("The speed at which the arm joints rotate.")]
    public float rotationSpeed = 50f;

    [Header("Gripper")]
    [Tooltip("The transform for the left finger of the gripper.")]
    public Transform gripperLeft;
    [Tooltip("The transform for the right finger of the gripper.")]
    public Transform gripperRight;
    [Tooltip("The speed at which the gripper opens and closes.")]
    public float gripperSpeed = 0.5f;
    [Tooltip("The minimum distance between the gripper fingers.")]
    public float minGripperGap = 0.1f;
    [Tooltip("The maximum distance between the gripper fingers.")]
    public float maxGripperGap = 1.0f;

    [Header("Grabbing")]
    [Tooltip("The key to press to grab or release an object.")]
    public KeyCode grabKey = KeyCode.Space;

    // --- Private Fields ---
    private Vector3 moveInput;
    private Transform heldObject = null; // The object currently being held
    private List<Collider> objectsInRange = new List<Collider>(); // Objects within the trigger zone

    void Update()
    {
        // --- Platform Input (Get in Update for responsiveness) ---
        moveInput = new Vector3(-Input.GetAxis("Vertical"), 0f, Input.GetAxis("Horizontal"));

        // --- Arm, Gripper, and Grabbing Input ---
        HandleArmMovement();
        //HandleGripperMovement();
        //HandleGrabbing();
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
            upperArm.Rotate(Vector3.forward, -rotationSpeed * Time.deltaTime);
        if (Input.GetKey(KeyCode.F))
            upperArm.Rotate(Vector3.forward, rotationSpeed * Time.deltaTime);

        // Forearm (T/G Keys)
        if (Input.GetKey(KeyCode.T))
            forearm.Rotate(Vector3.forward, -rotationSpeed * Time.deltaTime);
        if (Input.GetKey(KeyCode.G))
            forearm.Rotate(Vector3.forward, rotationSpeed * Time.deltaTime);

        // Gripper Base (Y/H Keys)
        if (Input.GetKey(KeyCode.Y))
            gripperBase.Rotate(Vector3.forward, -rotationSpeed * Time.deltaTime);
        if (Input.GetKey(KeyCode.H))
            gripperBase.Rotate(Vector3.forward, rotationSpeed * Time.deltaTime);
    }

    //private void HandleGripperMovement()
    //{
    //    float currentGap = Vector3.Distance(gripperLeft.position, gripperRight.position);

    //    // Close Gripper (Z Key)
    //    if (Input.GetKey(KeyCode.Z) && currentGap > minGripperGap)
    //    {
    //        gripperLeft.Translate(Vector3.right * gripperSpeed * Time.deltaTime);
    //        gripperRight.Translate(Vector3.left * gripperSpeed * Time.deltaTime);
    //    }

    //    // Open Gripper (X Key)
    //    if (Input.GetKey(KeyCode.X) && currentGap < maxGripperGap)
    //    {
    //        gripperLeft.Translate(Vector3.left * gripperSpeed * Time.deltaTime);
    //        gripperRight.Translate(Vector3.right * gripperSpeed * Time.deltaTime);
    //    }
    //}

    ///// <summary>
    ///// Handles the logic for grabbing and releasing objects.
    ///// </summary>
    //private void HandleGrabbing()
    //{
    //    // Check if the grab key is pressed down this frame
    //    if (Input.GetKeyDown(grabKey))
    //    {
    //        // If we are holding an object, release it.
    //        if (heldObject != null)
    //        {
    //            ReleaseObject();
    //        }
    //        // Otherwise, if we are not holding anything and there are objects in range, grab one.
    //        else if (objectsInRange.Any())
    //        {
    //            GrabObject();
    //        }
    //    }
    //}

    ///// <summary>
    ///// Attaches the closest valid object in range to the gripper.
    ///// </summary>
    //private void GrabObject()
    //{
    //    // Find the closest object from the list of objects in range
    //    Collider closestObject = objectsInRange
    //        .Where(obj => obj != null)
    //        .OrderBy(obj => Vector3.Distance(gripperBase.position, obj.transform.position))
    //        .FirstOrDefault();

    //    if (closestObject == null) return;

    //    heldObject = closestObject.transform;

    //    // Parent the object to the gripper base so it moves with the arm
    //    heldObject.SetParent(gripperBase);

    //    // Disable physics on the object while it's being held to prevent weird behavior
    //    Rigidbody rb = heldObject.GetComponent<Rigidbody>();
    //    if (rb != null)
    //    {
    //        rb.isKinematic = true;
    //    }
    //}

    ///// <summary>
    ///// Releases the currently held object.
    ///// </summary>
    //private void ReleaseObject()
    //{
    //    // Re-enable physics on the object so it can fall
    //    Rigidbody rb = heldObject.GetComponent<Rigidbody>();
    //    if (rb != null)
    //    {
    //        rb.isKinematic = false;
    //    }

    //    // Un-parent the object so it is free in the world again
    //    heldObject.SetParent(null);

    //    // Clear our reference to the held object
    //    heldObject = null;
    //}

    //// --- Unity Trigger Methods ---

    ///// <summary>
    ///// This is called by Unity's physics engine when a Collider enters the trigger zone.
    ///// </summary>
    //private void OnTriggerEnter(Collider other)
    //{
    //    // Check if the object has the "Obstacle" tag and isn't already in our list
    //    if (other.CompareTag("Obstacle") && !objectsInRange.Contains(other))
    //    {
    //        objectsInRange.Add(other);
    //        Debug.Log("Bruhwdklfhdghk,fgieryk");
    //    }
    //}

    ///// <summary>
    ///// This is called by Unity's physics engine when a Collider exits the trigger zone.
    ///// </summary>
    //private void OnTriggerExit(Collider other)
    //{
    //    // If the object that left is in our list, remove it
    //    if (objectsInRange.Contains(other))
    //    {
    //        objectsInRange.Remove(other);
    //    }
    //}
}

