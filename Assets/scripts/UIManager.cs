using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class UIManager : MonoBehaviour
{
    public AircraftPhysics currentPlane;
    Rigidbody currentRB;

    public TMP_Text text;

    void Awake()
    {
        currentRB = currentPlane.GetComponent<Rigidbody>();
    }

    void Update()
    {
        text.text = $"Speed: {currentRB.velocity.magnitude:0.00}\nFlaps: {currentPlane.flap}\nRaw Height: {currentPlane.transform.position.y:0.00}\nThrust: {currentPlane.thrust:0.00}\nBrake: {currentPlane.brakePercent:0.00}";
    }
}
