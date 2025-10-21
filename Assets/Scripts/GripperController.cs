using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class GripperController : MonoBehaviour
{

    [Header("Grabbing")]
    [Tooltip("The key to press to grab or release an object.")]
    public KeyCode grabKey = KeyCode.Space;

    // --- Private Fields ---
    private Transform heldObject = null;
    private List<Collider> objectsInRange = new List<Collider>();

    void Update()
    {
        HandleGrabbing();
    }

    private void HandleGrabbing()
    {
        if (Input.GetKeyDown(grabKey))
        {
            if (heldObject != null)
            {
                ReleaseObject();
            }
            else if (objectsInRange.Any())
            {
                GrabObject();
            }
        }
    }

    private void GrabObject()
    {
        Collider closestObject = objectsInRange
            .Where(obj => obj != null)
            .OrderBy(obj => Vector3.Distance(transform.position, obj.transform.position))
            .FirstOrDefault();

        if (closestObject == null) return;

        heldObject = closestObject.transform;
        heldObject.SetParent(transform); // Parent to this object (the gripperBase)

        Rigidbody rb = heldObject.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.isKinematic = true;
        }
    }

    private void ReleaseObject()
    {
        Rigidbody rb = heldObject.GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.isKinematic = false;
        }
        heldObject.SetParent(null);
        heldObject = null;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Obstacle") && !objectsInRange.Contains(other))
        {
            objectsInRange.Add(other);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (objectsInRange.Contains(other))
        {
            objectsInRange.Remove(other);
        }
    }
}
