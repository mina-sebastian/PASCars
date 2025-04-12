using UnityEngine;

public class FlyCamera : MonoBehaviour
{
    public float lookSpeed = 2.0f;
    public float moveSpeed = 10.0f;
    public float boostMultiplier = 2.5f;
    public bool invertY = false;

    private float yaw = 0.0f;
    private float pitch = 0.0f;

    void Update()
    {
        // Look rotation
        yaw += lookSpeed * Input.GetAxis("Mouse X");
        pitch -= lookSpeed * Input.GetAxis("Mouse Y") * (invertY ? -1 : 1);
        pitch = Mathf.Clamp(pitch, -90f, 90f);

        transform.eulerAngles = new Vector3(pitch, yaw, 0.0f);

        // Movement
        float speed = moveSpeed;
        if (Input.GetKey(KeyCode.LeftShift)) speed *= boostMultiplier;

        Vector3 move = new Vector3(
            Input.GetAxis("Horizontal"),
            Input.GetKey(KeyCode.E) ? 1 : Input.GetKey(KeyCode.Q) ? -1 : 0,
            Input.GetAxis("Vertical")
        );

        transform.position += transform.TransformDirection(move) * speed * Time.deltaTime;
    }

    void OnEnable()
    {
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    void OnDisable()
    {
        Cursor.lockState = CursorLockMode.None;
        Cursor.visible = true;
    }
}
