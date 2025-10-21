using UnityEngine;

public class LCDlogic : MonoBehaviour
{
    [Header("Activation Settings")]
    [Tooltip("Assign the GameObject that you want to set active when the player enters.")]
    public GameObject objectToActivate;

    void Start()
    {
        // Check if the objectToActivate is assigned in the Inspector to avoid errors.
        if (objectToActivate == null)
        {
            Debug.LogError("Object to Activate is not assigned in the LCDlogic script on " + gameObject.name);
        }
        else
        {
            // Ensure the object is inactive at the start of the game.
            objectToActivate.SetActive(false);
        }
    }

    /// <summary>
    /// This method is called by Unity's physics engine when a Collider enters the trigger zone.
    /// </summary>
    /// <param name="other">The Collider of the object that entered the trigger.</param>
    private void OnTriggerEnter(Collider other)
    {
        // Check if the entering object has the tag "Player" and if the object to activate is assigned.
        if (objectToActivate != null && other.CompareTag("Player"))
        {
            // Set the specified GameObject to be active.
            objectToActivate.SetActive(true);
            Debug.Log(other.name + " entered the LCD trigger. Activating " + objectToActivate.name);
        }
    }

    /// <summary>
    /// Optional: This method deactivates the object when the player leaves the trigger zone.
    /// </summary>
    private void OnTriggerExit(Collider other)
    {
        // Check if the exiting object has the tag "Player".
        if (objectToActivate != null && other.CompareTag("Player"))
        {
            // Deactivate the object again.
            objectToActivate.SetActive(false);
            Debug.Log(other.name + " exited the LCD trigger. Deactivating " + objectToActivate.name);
        }
    }
}
