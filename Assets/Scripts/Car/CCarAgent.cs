using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Barmetler.RoadSystem; // For RoadSystemNavigator

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(RoadSystemNavigator))]
public class CCarAgent : Agent
{
    private Rigidbody rb;
    private RoadSystemNavigator navigator;

    [Header("Car Settings")]
    public float maxSteerAngle = 30f;
    public float motorForce = 1500f;

    [Header("Rewards")]
    public float centerlinePenaltyWeight = 0.01f;

    [Header("Driving Direction")]
    public Vector3 desiredForward; // will be set at spawn

    private float steerInput;
    private float motorInput;
    private float lateralOffsetCached = 0f;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();

        // Auto-assign navigator
        navigator = GetComponent<RoadSystemNavigator>();
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Debug.Log("A");
        // 1. Forward velocity (normalized)
        float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
        sensor.AddObservation(Mathf.Clamp(forwardSpeed / 20f, -1f, 1f));

        // 2. Lateral offset from centerline
        if (navigator != null && navigator.CurrentPoints != null && navigator.CurrentPoints.Count > 0)
        {
            Vector3 nearestPoint = FindClosestPoint(transform.position);
            Vector3 localOffset = transform.InverseTransformPoint(nearestPoint);
            lateralOffsetCached = localOffset.x;
            sensor.AddObservation(Mathf.Clamp(lateralOffsetCached / 5f, -1f, 1f));
        }
        else
        {
            sensor.AddObservation(0f);
            lateralOffsetCached = 0f;
        }

        // 3. Alignment with desiredForward
        float alignment = Vector3.Dot(transform.forward, desiredForward.normalized);
        sensor.AddObservation(alignment);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        steerInput = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        motorInput = Mathf.Clamp(actions.ContinuousActions[1], 0f, 1f);

        ApplyMovement();
        EvaluateRewards();
    }

    private void ApplyMovement()
    {
        float steer = steerInput * maxSteerAngle;
        float motor = motorInput * motorForce;

        transform.Rotate(0f, steer * Time.fixedDeltaTime, 0f);
        rb.AddForce(transform.forward * motor * Time.fixedDeltaTime);
    }

    private void EvaluateRewards()
    {
        // Positive reward for forward movement
        float forwardVelocity = Vector3.Dot(rb.linearVelocity, desiredForward.normalized);
        AddReward(0.1f * (forwardVelocity / 10f)); // normalize ~10m/s max

        // Negative reward for being off center
        AddReward(-centerlinePenaltyWeight * Mathf.Abs(lateralOffsetCached));

        // Death penalty for being off road
        if (!IsOnRoad())
        {
            AddReward(-1.0f);
            EndEpisode();
        }
    }

    private bool IsOnRoad()
    {
        return Physics.Raycast(transform.position + Vector3.up * 0.5f, Vector3.down, 2f, LayerMask.GetMask("Road"));
    }

    private Vector3 FindClosestPoint(Vector3 position)
    {
        float minDist = float.MaxValue;
        Vector3 closest = Vector3.zero;

        foreach (var point in navigator.CurrentPoints)
        {
            float dist = Vector3.Distance(position, point.position);
            if (dist < minDist)
            {
                minDist = dist;
                closest = point.position;
            }
        }

        return closest;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal"); // steer
        continuousActionsOut[1] = Input.GetAxis("Vertical");   // gas
    }
}
