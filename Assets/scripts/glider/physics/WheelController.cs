using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class WheelController : MonoBehaviour
{
    AircraftPhysics plane;
    PlayerInput input;
    WheelCollider wheel;
    [SerializeField] bool isSteerable = false;
    [SerializeField] float maxSteerAngle = 30;
    [SerializeField] float brakeTorque = 10000;
    float brakePercent = 0;
    float steerAngle = 0;
    public bool weightOnWheels = false;

    void Awake()
    {
        plane = GetComponentInParent<AircraftPhysics>();
        input = GetComponentInParent<PlayerInput>();
        wheel = GetComponent<WheelCollider>();
    }

    void Update()
    {
        if (!weightOnWheels) return;

        if (input.currentActionMap == input.actions.FindActionMap("flight"))
        {
            steerAngle = input.actions.FindAction("yaw").ReadValue<float>() * maxSteerAngle;
            brakePercent = input.actions.FindAction("brake").ReadValue<float>();
        }
    }

    void FixedUpdate()
    {
        WheelHit hit;
        if (wheel.GetGroundHit(out hit))
            weightOnWheels = hit.collider.gameObject.CompareTag("ground");
        else
            weightOnWheels = false;

        if (!weightOnWheels) return;

        wheel.brakeTorque = brakePercent * brakeTorque;

        if (plane.thrust > 0)
        {
            wheel.motorTorque = 1;
        }
        if (isSteerable)
            wheel.steerAngle = steerAngle;
    }
}
