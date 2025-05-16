using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Barmetler.RoadSystem;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(DecisionRequester))]
public class CarAgent : Agent
{
    Rigidbody rb;

    [Header("Train Settings")]
    public int maxRelapses = 5;
    public float maxDistanceFromRoad = 3.0f;

    [Header("Car Settings")]
    public float maxSteerAngle = 30f;
    public float motorForce = 1500f;
    public float maxSpeed = 20f;
    public float naturalDrag = 0.75f;

    private RoadSystemNavigator navigator;

    private List<Vector3> _fixedPath = new List<Vector3>();
    private int _previousPathIndex = 0;
    private Vector3 _currentWaypoint;
    private int _nbOfRelapses = 0;
    private int _speedZero = 0;

    public override void Initialize()
    {
        navigator = GetComponent<RoadSystemNavigator>();

        rb = GetComponent<Rigidbody>();
        rb.linearDamping = naturalDrag;
        rb.angularDamping = 0.05f;
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        transform.localPosition = new Vector3(-83.15f, 0.67f, 23.6f);
        transform.rotation = Quaternion.identity;

        navigator.Goal = new Vector3(100, 0, 100); // or auto-select valid goal
        navigator.CalculateWayPointsSync();

        // Try waiting manually for CurrentPoints
        for (int i = 0; i < 30; i++) // max ~0.5 sec
        {
            if (navigator.CurrentPoints.Count > 0)
                break;
        }

        if (navigator.CurrentPoints.Count == 0)
        {
            Debug.LogError("[CarAgent] No waypoints available after waiting. Ending episode.");
            EndEpisode();
            return;
        }

        _fixedPath.Clear();
        _previousPathIndex = 0;
        _nbOfRelapses = 0;
        _speedZero = 0;

        _currentWaypoint = navigator.CurrentPoints[0].position;
        for (int i = 1; i < navigator.CurrentPoints.Count; i++)
        {
            _fixedPath.Add(navigator.CurrentPoints[i].position);
        }

        Debug.Log($"[CarAgent] OnEpisodeBegin complete. Waypoints count: {_fixedPath.Count}");
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        float speed = rb.linearVelocity.magnitude / maxSpeed;
        sensor.AddObservation(speed);

        if (speed < 0.02f)
            _speedZero++;
        else
            _speedZero = 0;

        if (navigator.CurrentPoints.Count > 0)
        {
            _currentWaypoint = navigator.CurrentPoints[0].position;
            float closestDist = Vector3.Distance(_currentWaypoint, rb.position) / maxDistanceFromRoad;
            sensor.AddObservation(closestDist);

            // Velocity in local space
            Vector3 localVelocity = transform.InverseTransformDirection(rb.linearVelocity);
            sensor.AddObservation(localVelocity.x / maxSpeed);
            sensor.AddObservation(localVelocity.z / maxSpeed);

            // Add direction to lookahead waypoints (1–3 ahead)
            int wpCount = Mathf.Min(4, navigator.CurrentPoints.Count);
            for (int i = 1; i < wpCount; i++)
            {
                Vector3 toNext = (navigator.CurrentPoints[i].position - transform.position).normalized;
                Vector3 localToNext = transform.InverseTransformDirection(toNext);
                sensor.AddObservation(localToNext.x);
                sensor.AddObservation(localToNext.z);
            }

            // Add forward alignment with current waypoint
            Vector3 dirToCurrent = (_currentWaypoint - transform.position).normalized;
            float headingAlignment = Vector3.Dot(transform.forward, dirToCurrent);
            sensor.AddObservation(headingAlignment); // 1 = perfectly aligned, -1 = opposite

            Debug.Log($"[OBS] Speed: {speed:F2} | Dist: {closestDist:F2} | LocalV: ({localVelocity.x:F2}, {localVelocity.z:F2}) | HeadingAlign: {headingAlignment:F2}");
        }
        else
        {
            // Fallbacks in case of invalid state
            sensor.AddObservation(1f);  // Closest distance
            sensor.AddObservation(0f);  // Velocity x
            sensor.AddObservation(0f);  // Velocity z
            sensor.AddObservation(0f);  // Lookahead x
            sensor.AddObservation(0f);  // Lookahead z
            sensor.AddObservation(0f);  // Heading alignment

            Debug.LogWarning("[OBSERVATIONS] No current waypoints available!");
        }
    }



    public override void OnActionReceived(ActionBuffers actions)
    {
        // Discrete branch 0: steer {0=left,1=none,2=right}
        int steerAction = actions.DiscreteActions[0];
        float steerInput = steerAction == 0 ? -1f
                         : steerAction == 2 ? 1f
                                            : 0f;

        // Discrete branch 1: throttle {0=none,1=forward,2=reverse}
        int motorAction = actions.DiscreteActions[1];
        float motorInput = motorAction == 1 ? 1f
                         : motorAction == 2 ? -1f
                                            : 0f;

        ApplyMotor(motorInput);
        ApplySteering(steerInput);

        float closestDist = Vector3.Distance(_currentWaypoint, rb.position) / maxDistanceFromRoad;

        RepositionCurrentPositionIndex(closestDist);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Map keyboard to the same discrete actions
        var da = actionsOut.DiscreteActions;
        float h = Input.GetAxis("Horizontal");
        da[0] = h < -0.5f ? 0 : h > 0.5f ? 2 : 1;

        float v = Input.GetAxis("Vertical");
        da[1] = v > 0.5f ? 1
              : v < -0.5f ? 2
                          : 0;
    }

    private void ApplyMotor(float motorInput)
    {
        Vector3 force = transform.forward * motorInput * motorForce;

        // Only add force if under max speed, or if reversing
        if (rb.linearVelocity.magnitude < maxSpeed || Vector3.Dot(rb.linearVelocity, transform.forward) < 0)
        {
            rb.AddForce(force, ForceMode.Force);
        }
        // No manual braking natural drag handles it!
    }

    private void ApplySteering(float steerInput)
    {
        float currentSpeed = rb.linearVelocity.magnitude;
        float speedFactor = Mathf.Clamp01(currentSpeed / 10f);

        float steerAngle = steerInput * maxSteerAngle * speedFactor;
        Quaternion steerRotation = Quaternion.Euler(0f, steerAngle * Time.fixedDeltaTime, 0f);

        rb.MoveRotation(rb.rotation * steerRotation);
    }

    private int searchRadius = 10;
    private void RepositionCurrentPositionIndex(float closestDist)
    {
        if (_fixedPath.Count == 0) return;

        if(_speedZero > 40)
        {
            _speedZero = 0;
            EndEpisode();
            Debug.LogWarning("Stopped");
        }

        if(closestDist >= 1)
        {
            AddReward(-1);
            EndEpisode();
            Debug.LogWarning("Out of the road");
        }

        int bestIndex = _previousPathIndex;
        float bestDistance = float.MaxValue;

        // Only check a sliding window ahead and behind to avoid jitter


        int start = Mathf.Max(0, _previousPathIndex - searchRadius);
        int end = Mathf.Min(_fixedPath.Count, _previousPathIndex + searchRadius);

        for (int i = start; i < end; i++)
        {
            float dist = Vector3.Distance(_currentWaypoint, _fixedPath[i]);
            if (dist < bestDistance)
            {
                bestDistance = dist;
                bestIndex = i;
            }
        }

        int delta = bestIndex - _previousPathIndex;

/*        float roadCenterFactor = 1f - Mathf.Clamp01(closestDist); // 1 = perfect, 0 = far
        AddReward(roadCenterFactor * 0.0005f);*/

        if (delta > 0)
        {
            float roadCenterFactor = (1f - Mathf.Clamp01(closestDist)) * 2; // 1 = perfect, 0 = far
            roadCenterFactor = Mathf.Clamp01(roadCenterFactor);
            float progressReward = delta * 0.15f * roadCenterFactor;
            AddReward(progressReward);
        }
        else if (delta < 0)
        {
            AddReward(delta * 0.4f); // Penalty for regression (stronger)
            _nbOfRelapses += 1;
        }

        _previousPathIndex = bestIndex;

        if(_nbOfRelapses > maxRelapses)
        {
            EndEpisode();
            Debug.LogWarning($"Relapsed more than {maxRelapses} times");
        }

        /*Debug.Log($"[PATH PROGRESS] Index: {bestIndex} | Δ: {delta}");*/
    }


    private void OnDrawGizmos()
    {
        for (int i = 0; i < _fixedPath.Count; i++)
        {
            if (i == _previousPathIndex)
            {
                Gizmos.color = Color.green;
            }
            else
            {
                Gizmos.color = Color.grey;
            }
            Gizmos.DrawSphere(_fixedPath[i], 0.2f);
        }

        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(_currentWaypoint, 0.25f);
    }


}
