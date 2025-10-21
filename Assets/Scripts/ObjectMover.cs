using UnityEngine;
using System.Collections; // Required for using Coroutines

public class ObjectMover : MonoBehaviour
{
    [Header("Movement Settings")]
    [Tooltip("The GameObject that will be moved.")]
    public Transform objectToMove;

    [Tooltip("The distance and direction the object will move from its starting point.")]
    public Vector3 moveDistance = new Vector3(0, 0.25f, 0);

    [Tooltip("How long the movement should take in seconds.")]
    public float moveDuration = 3.0f;

    // --- Private Fields ---
    private Vector3 startPosition;
    private Vector3 endPosition;
    private bool hasMoved = false; // Prevents the trigger from firing multiple times

    void Start()
    {
        // Check if the object to move is assigned
        if (objectToMove == null)
        {
            Debug.LogError("Object to Move is not assigned in the ObjectMover script on " + gameObject.name);
            return; // Disable the script if nothing is assigned
        }

        // Store the initial positions
        startPosition = objectToMove.position;
        endPosition = startPosition + moveDistance;
    }

    /// <summary>
    /// This method is called by Unity's physics engine when a Collider enters the trigger zone.
    /// </summary>
    private void OnTriggerEnter(Collider other)
    {
        // Check if the entering object is the Player and if the object hasn't already moved
        if (other.CompareTag("Player") && !hasMoved)
        {
            Debug.Log("Player entered the trigger. Starting movement.");
            // Start the coroutine to handle the movement over time
            StartCoroutine(MoveObjectCoroutine());
            hasMoved = true; // Set flag to true so it only runs once
        }
    }

    /// <summary>
    /// A coroutine that smoothly moves the object from its start to end position over a set duration.
    /// </summary>
    private IEnumerator MoveObjectCoroutine()
    {
        float elapsedTime = 0f;

        while (elapsedTime < moveDuration)
        {
            // Calculate the new position using linear interpolation (Lerp)
            objectToMove.position = Vector3.Lerp(startPosition, endPosition, (elapsedTime / moveDuration));

            // Increment the elapsed time by the time passed since the last frame
            elapsedTime += Time.deltaTime;

            // Wait for the next frame before continuing the loop
            yield return null;
        }

        // Ensure the object is exactly at the end position when the loop finishes
        objectToMove.position = endPosition;
        Debug.Log("Movement complete.");
    }
}
