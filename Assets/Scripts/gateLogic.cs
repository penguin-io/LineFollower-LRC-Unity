using UnityEngine;
using TMPro; // Required for using TextMeshPro components

public class gateLogic : MonoBehaviour
{
    [Header("UI Settings")]
    [Tooltip("Assign the TextMeshPro UI object you want to change the color of.")]
    public TextMeshPro statusText;

    [Tooltip("The color the text will change to when the robot enters.")]
    public Color activeColor = Color.red;

    private Color originalColor; // To store the text's original color

    void Start()
    {
        // Check if the statusText is assigned to avoid errors
        if (statusText != null)
        {
            // Store the original color of the text so we can potentially reset it later
            originalColor = statusText.color;
        }
        else
        {
            Debug.LogError("Status Text is not assigned in the gateLogic script on " + gameObject.name);
        }
    }

    /// <summary>
    /// This method is called by Unity's physics engine when a Collider enters the trigger zone.
    /// </summary>
    /// <param name="other">The Collider of the object that entered the trigger.</param>
    private void OnTriggerEnter(Collider other)
    {
        // Check if the entering object has the tag "robo" and if the text object is assigned
        if (statusText != null && other.CompareTag("Player"))
        {
            // Change the color of the TextMeshPro object to the active color
            statusText.color = activeColor;
            Debug.Log(other.name + " entered the gate. Status text changed to red.");
        }
    }
}
