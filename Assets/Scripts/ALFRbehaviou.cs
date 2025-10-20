using UnityEngine;

public class ALFRbehaviou : MonoBehaviour
{

    private float horiInput = 0;
    private float veriInput = 0;
    [SerializeField] private float speed = 5f;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        horiInput = Input.GetAxis("Horizontal");    
        veriInput = Input.GetAxis("Vertical");
        transform.Translate(new Vector3(horiInput * Time.deltaTime * speed, 0, veriInput * Time.deltaTime * speed));
    }
}
