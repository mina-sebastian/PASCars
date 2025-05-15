using UnityEngine;
using Barmetler.RoadSystem;

public class CarSpawner : MonoBehaviour
{
    [Header("Spawner Settings")]
    public GameObject carPrefab;
    public RoadSystem roadSystem;   // Your big MB Road System
    public float goalDistanceAhead = 1000f; // How far forward to set Goal

    public void SpawnCar()
    {
        if (carPrefab == null || roadSystem == null)
        {
            Debug.LogError("Car Prefab or Road System not assigned!");
            return;
        }

        // Spawn at this GameObject's position and rotation
        Vector3 spawnPosition = transform.position;
        Vector3 spawnForward = transform.forward;

        // Instantiate car
        GameObject carObj = Instantiate(carPrefab, spawnPosition, Quaternion.LookRotation(spawnForward, Vector3.up));

        // Setup CarAgent
        var agent = carObj.GetComponent<CCarAgent>();
        if (agent != null)
        {
            agent.desiredForward = spawnForward;
        }

        // Setup RoadSystemNavigator
        var navigator = carObj.GetComponent<RoadSystemNavigator>();
        if (navigator != null)
        {
            navigator.currentRoadSystem = roadSystem;
            navigator.Goal = spawnPosition + spawnForward * goalDistanceAhead;
            navigator.CalculateWayPointsSync(); // Force it to calculate immediately
        }
    }

    void Start()
    {
        SpawnCar();
    }
}
