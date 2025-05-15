using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Barmetler.RoadSystem; // Needed for RoadSystemNavigator

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(RoadSystemNavigator))]
public class CarAgent : Agent
{
    private Rigidbody rb;
    private RoadSystemNavigator navigator;

    [Header("Car Settings")]
    public float maxSteerAngle = 30f;
    public float motorForce = 1500f;
    public float maxSpeed = 20f;
    public float naturalDrag = 0.75f;

    private float steerInput;
    private float motorInput;

    private Vector3 lastPosition;
    private float timeSinceLastMove;

    public override void Initialize()
    {
        Debug.Log("[CarAgent] Initialize() called.");

        rb = GetComponent<Rigidbody>();
        navigator = GetComponent<RoadSystemNavigator>();

        rb.linearDamping = naturalDrag;
        rb.angularDamping = 0.05f;

        lastPosition = transform.position;

        Debug.Log("[CarAgent] Rigidbody and Navigator initialized successfully.");
    }

    public override void OnEpisodeBegin()
    {
        Debug.Log("[CarAgent] OnEpisodeBegin() called.");

        // Reset car physics
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        //Debug.Log("[CarAgent] Rigidbody velocities reset.");

        // Reset position to spawn
        var spawn = GameObject.Find("CarSpawn1");
        if (spawn != null)
        {
            transform.position = spawn.transform.position;
            transform.rotation = spawn.transform.rotation;
            //Debug.Log("[CarAgent] Car repositioned to CarSpawn1.");
        }
        else
        {
            Debug.LogWarning("[CarAgent] WARNING: CarSpawn1 not found!");
        }

        //Debug.Log("[CarAgent] Starting navigator waypoint recalculation...");
        navigator.CalculateWayPointsSync();
        //Debug.Log("[CarAgent] Navigator waypoints recalculated.");
    }

    private void AddRaycastObservations(VectorSensor sensor)
    {
        float rayLength = 10f;
        Vector3[] directions = new Vector3[]
        {
            transform.forward,
            -transform.forward,
            transform.right,
            -transform.right,
            (transform.forward + transform.right).normalized,
            (transform.forward - transform.right).normalized
        };

        foreach (var dir in directions)
        {
            if (Physics.Raycast(transform.position, dir, out RaycastHit hit, rayLength))
            {
                sensor.AddObservation(hit.distance / rayLength);
            }
            else
            {
                sensor.AddObservation(1f);
            }
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Debug.Log("[CarAgent] CollectObservations() called.");

        Vector3 localVelocity = transform.InverseTransformDirection(rb.linearVelocity);
        sensor.AddObservation(localVelocity.x);
        sensor.AddObservation(localVelocity.z);

        if (navigator.CurrentPoints.Count > 0)
        {
            Vector3 nextPoint = navigator.CurrentPoints[0].position;
            Vector3 toNext = transform.InverseTransformPoint(nextPoint);
            sensor.AddObservation(toNext.normalized);
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            Debug.LogWarning("[CarAgent] WARNING: No navigator points available.");
        }

        AddRaycastObservations(sensor);

        sensor.AddObservation(steerInput);
        sensor.AddObservation(motorInput);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        steerInput = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        motorInput = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        Debug.Log($"{steerInput} {motorInput}");

        ApplyMotor();
        ApplySteering();

        AddReward(0.001f * motorInput);

        if (Vector3.Distance(transform.position, lastPosition) < 0.5f)
        {
            timeSinceLastMove += Time.deltaTime;
            if (timeSinceLastMove > 20f)
            {
                AddReward(-0.5f);
                Debug.LogWarning("[CarAgent] Penalty: Car stuck. Ending episode.");
                EndEpisode();
            }
        }
        else
        {
            timeSinceLastMove = 0f;
            lastPosition = transform.position;
        }

        if (navigator.CurrentPoints.Count > 0)
        {
            float lateralOffset = Vector3.Distance(transform.position, navigator.CurrentPoints[0].position);
            AddReward(-lateralOffset * 0.001f);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        Debug.LogWarning("[CarAgent] INFO: Heuristic received");
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }

    private void ApplyMotor()
    {
        Vector3 force = transform.forward * motorInput * motorForce;
        if (rb.linearVelocity.magnitude < maxSpeed || Vector3.Dot(rb.linearVelocity, transform.forward) < 0)
        {
            rb.AddForce(force, ForceMode.Force);
        }
    }

    private void ApplySteering()
    {
        float currentSpeed = rb.linearVelocity.magnitude;
        float speedFactor = Mathf.Clamp01(currentSpeed / 10f);

        float steerAngle = steerInput * maxSteerAngle * speedFactor;
        Quaternion steerRotation = Quaternion.Euler(0f, steerAngle * Time.fixedDeltaTime, 0f);

        rb.MoveRotation(rb.rotation * steerRotation);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Car"))
        {
            AddReward(-1.0f);
            Debug.LogWarning("[CarAgent] Collision with another car detected. Ending episode.");
            EndEpisode();
        }
    }
}
