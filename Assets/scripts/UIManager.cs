using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class UIManager : MonoBehaviour
{
    public AircraftPhysics currentPlane;
    Rigidbody currentRB;
    WheelController[] wheels;

    public TMP_Text text;

    void Awake()
    {
        currentRB = currentPlane.GetComponent<Rigidbody>();
        wheels = currentPlane.GetComponentsInChildren<WheelController>();
    }

    void Update()
    {
        bool anyWeightOnWheels = false;
        foreach (WheelController wheel in wheels)
        {
            anyWeightOnWheels |= wheel.weightOnWheels;
        }
        text.text = $"Speed: {currentRB.velocity.magnitude:0.00}\nFlaps: {currentPlane.flap}\nRaw Height: {currentPlane.transform.position.y:0.00}\nThrust: {currentPlane.thrust:0.00}\nSpeed Brake: {currentPlane.speedBrake:0.00}\nWoW: {anyWeightOnWheels}";
    }
}
