using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody), typeof(PlayerInput))]
public class AircraftPhysics : MonoBehaviour
{
    const float PREDICTION_TIMESTEP_FRACTION = 0.5f;

    public List<AeroSurface> aerodynamicSurfaces = null;
    List<AeroSurface> controlSurfaces = new List<AeroSurface>();

    public float dragAmount = 100;
    public float thrust;
    float pitch, yaw, roll;

    Vector3 wind = Vector3.zero;
    Rigidbody rb;
    PlayerInput input;
    BiVector3 currentForceAndTorque;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        input = GetComponent<PlayerInput>();
        for (int i = 0; i < aerodynamicSurfaces.Count; i++)
        {
            if (aerodynamicSurfaces[i].IsControlSurface)
            {
                controlSurfaces.Add(aerodynamicSurfaces[i]);
            }
        }
    }

    private void Update()
    {
        if (input.currentActionMap == input.actions.FindActionMap("flight"))
        {
            Vector2 pitchRoll = input.actions.FindAction("attitude").ReadValue<Vector2>();

            roll = pitchRoll.x;
            pitch = pitchRoll.y;
            yaw = input.actions.FindAction("yaw").ReadValue<float>();
            thrust = input.actions.FindAction("thrust").ReadValue<float>() * 10000;
        }
        else
        {
            roll = 0;
            pitch = 0;
            yaw = 0;
            thrust = 0;
        }

        foreach (AeroSurface controlSurface in controlSurfaces)
        {
            switch (controlSurface.InputType)
            {
                case ControlInputType.Pitch:
                    controlSurface.SetFlapAngle(pitch * controlSurface.maxAngle);
                    break;
                case ControlInputType.Roll:
                    controlSurface.SetFlapAngle(roll * controlSurface.maxAngle);
                    break;
                case ControlInputType.Yaw:
                    controlSurface.SetFlapAngle(yaw * controlSurface.maxAngle);
                    break;
            }
        }
    }

    private void FixedUpdate()
    {
        BiVector3 forceAndTorqueThisFrame =
            CalculateAerodynamicForces(rb.velocity, rb.angularVelocity, wind, 1.2f, rb.worldCenterOfMass);

        Vector3 velocityPrediction = PredictVelocity(forceAndTorqueThisFrame.p
            + transform.forward * thrust + Physics.gravity * rb.mass);
        Vector3 angularVelocityPrediction = PredictAngularVelocity(forceAndTorqueThisFrame.q);

        BiVector3 forceAndTorquePrediction =
            CalculateAerodynamicForces(velocityPrediction, angularVelocityPrediction, wind, 1.2f, rb.worldCenterOfMass);

        currentForceAndTorque = (forceAndTorqueThisFrame + forceAndTorquePrediction) * 0.5f;

        if (float.IsNaN(currentForceAndTorque.p.x)) currentForceAndTorque.p = Vector3.zero;
        if (float.IsNaN(currentForceAndTorque.q.x)) currentForceAndTorque.q = Vector3.zero;

        rb.AddForce(currentForceAndTorque.p);
        rb.AddTorque(currentForceAndTorque.q);

        rb.AddForce(transform.forward * thrust);
    }

    private BiVector3 CalculateAerodynamicForces(Vector3 velocity, Vector3 angularVelocity, Vector3 wind, float airDensity, Vector3 centerOfMass)
    {
        BiVector3 forceAndTorque = new BiVector3();
        foreach (AeroSurface surface in aerodynamicSurfaces)
        {
            Vector3 relativePosition = surface.transform.position - centerOfMass;
            forceAndTorque += surface.CalculateForces(-velocity + wind
                - Vector3.Cross(angularVelocity,
                relativePosition),
                airDensity, relativePosition);
        }
        return forceAndTorque;
    }

    private Vector3 PredictVelocity(Vector3 force)
    {
        return rb.velocity + Time.fixedDeltaTime * PREDICTION_TIMESTEP_FRACTION * force / rb.mass;
    }

    private Vector3 PredictAngularVelocity(Vector3 torque)
    {
        Quaternion inertiaTensorWorldRotation = rb.rotation * rb.inertiaTensorRotation;
        Vector3 torqueInDiagonalSpace = Quaternion.Inverse(inertiaTensorWorldRotation) * torque;
        Vector3 angularVelocityChangeInDiagonalSpace;
        angularVelocityChangeInDiagonalSpace.x = torqueInDiagonalSpace.x / rb.inertiaTensor.x;
        angularVelocityChangeInDiagonalSpace.y = torqueInDiagonalSpace.y / rb.inertiaTensor.y;
        angularVelocityChangeInDiagonalSpace.z = torqueInDiagonalSpace.z / rb.inertiaTensor.z;

        return rb.angularVelocity + Time.fixedDeltaTime * PREDICTION_TIMESTEP_FRACTION
            * (inertiaTensorWorldRotation * angularVelocityChangeInDiagonalSpace);
    }


#if UNITY_EDITOR
    // For gizmos drawing.
    public void CalculateCenterOfLift(out Vector3 center, out Vector3 force, Vector3 displayAirVelocity, float displayAirDensity)
    {
        Vector3 com;
        BiVector3 forceAndTorque;
        if (aerodynamicSurfaces == null)
        {
            center = Vector3.zero;
            force = Vector3.zero;
            return;
        }

        if (rb == null)
        {
            com = GetComponent<Rigidbody>().worldCenterOfMass;
            forceAndTorque = CalculateAerodynamicForces(-displayAirVelocity, Vector3.zero, Vector3.zero, displayAirDensity, com);
        }
        else
        {
            com = rb.worldCenterOfMass;
            forceAndTorque = currentForceAndTorque;
        }

        force = forceAndTorque.p;
        center = com + Vector3.Cross(forceAndTorque.p, forceAndTorque.q) / forceAndTorque.p.sqrMagnitude;
    }
#endif
}
