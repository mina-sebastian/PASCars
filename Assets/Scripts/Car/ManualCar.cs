using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class ManualCarController : MonoBehaviour
{
    private Rigidbody rb;

    [Header("Car Settings")]
    public float maxSteerAngle = 30f;
    public float motorForce = 1500f;
    public float maxSpeed = 20f;
    public float naturalDrag = 0.75f; // How much the car slows down naturally

    private float steerInput;
    private float motorInput;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.linearDamping = naturalDrag;
        rb.angularDamping = 0.05f; // Small rotational drag for stability
    }

    private void Update()
    {
        // Get player input
        steerInput = Input.GetAxis("Horizontal"); // A/D or Left/Right Arrows
        motorInput = Input.GetAxis("Vertical");   // W/S or Up/Down Arrows
    }

    private void FixedUpdate()
    {
        ApplyMotor();
        ApplySteering();
    }

    private void ApplyMotor()
    {
        Vector3 force = transform.forward * motorInput * motorForce;

        // Only add force if under max speed, or if reversing
        if (rb.linearVelocity.magnitude < maxSpeed || Vector3.Dot(rb.linearVelocity, transform.forward) < 0)
        {
            rb.AddForce(force, ForceMode.Force);
        }
        // No manual braking � natural drag handles it!
    }

    private void ApplySteering()
    {
        float currentSpeed = rb.linearVelocity.magnitude;
        float speedFactor = Mathf.Clamp01(currentSpeed / 10f);

        float steerAngle = steerInput * maxSteerAngle * speedFactor;
        Quaternion steerRotation = Quaternion.Euler(0f, steerAngle * Time.fixedDeltaTime, 0f);

        rb.MoveRotation(rb.rotation * steerRotation);
    }
}
