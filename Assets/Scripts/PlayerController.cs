using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerController : MonoBehaviour
{
    public int maxRPM = 2500;
    [SerializeField] int wingArea = 10;
    [SerializeField] float aspectRatio = 5;
    Rigidbody2D rb;
    PlayerInput input;
    [SerializeField] Camera cam;
    public float throttle = 0f;
    public int rpm = 0;

    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody2D>();
        input = GetComponent<PlayerInput>();
    }

    // Update is called once per frame
    void Update()
    {
        cam.transform.position = new Vector3(transform.position.x, transform.position.y, -10);

        if (input.actions["pitch up"].ReadValue<float>() > 0)
        {
            transform.Rotate(0, 0, 1);
        }
        else if (input.actions["pitch down"].ReadValue<float>() > 0)
        {
            transform.Rotate(0, 0, -1);
        }
        if (input.actions["throttle up"].ReadValue<float>() > 0)
        {
            throttle += 0.01f;
        }
        else if (input.actions["throttle down"].ReadValue<float>() > 0)
        {
            throttle -= 0.01f;
        }
        throttle = Mathf.Clamp(throttle, 0, 1);
    }

    void FixedUpdate()
    {
        float AoA = Vector2.Angle(transform.right, rb.velocity) * Mathf.Deg2Rad;
        float liftCoefficient = AoA - (-5 * Mathf.Deg2Rad);
        // TODO velocity needs to be velocity over the wings (ie parallel to the wings)
        float lift = liftCoefficient * rb.velocity.sqrMagnitude * 0.5f * wingArea;
        float drag = (Mathf.Pow(liftCoefficient, 2) / (Mathf.PI * aspectRatio)) * rb.velocity.sqrMagnitude * 0.5f * wingArea;
        rpm = (int)(throttle * maxRPM);
        rb.AddForce(transform.up * lift);
        rb.AddForce(transform.right * rpm);
        rb.AddForce(-transform.right * drag);
    }
}
