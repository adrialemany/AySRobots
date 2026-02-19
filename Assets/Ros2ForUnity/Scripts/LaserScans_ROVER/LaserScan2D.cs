using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using sensor_msgs.msg;

public class LaserScan2D : MonoBehaviour
{
    public string topicName = "/scan";
    public Material pointMaterial;
    public int messageBufferSize = 10; // Number of messages to accumulate
    public float pointScale = 0.05f; // Scaling factor for the points

    private ConcurrentQueue<Action> mainThreadActions = new ConcurrentQueue<Action>();
    private ROS2Node ros2Node;
    private Subscription<LaserScan> scanSubscription;
    private Mesh pointCloudMesh;

    private Queue<LaserScan> scanQueue = new Queue<LaserScan>();
    private Vector3[] vertices;
    private Color[] colors;
    private int[] indices;

    void Start()
    {
        var ros2UnityComponent = FindObjectOfType<ROS2UnityComponent>();
        ros2Node = ros2UnityComponent.CreateNode("laser_scan_subscriber");

        scanSubscription = ros2Node.CreateSubscription<LaserScan>(topicName, ScanCallback);

        pointCloudMesh = new Mesh();
        GetComponent<MeshFilter>().mesh = pointCloudMesh;
    }

    private void ScanCallback(LaserScan scan)
    {
        mainThreadActions.Enqueue(() =>
        {
            if (scanQueue.Count >= messageBufferSize)
            {
                scanQueue.Dequeue();
            }

            scanQueue.Enqueue(scan);
            ProcessScanQueue();
        });
    }

    private void ProcessScanQueue()
    {
        int totalPoints = 0;
        foreach (var scan in scanQueue)
        {
            totalPoints += scan.Ranges.Length;
        }

        if (vertices == null || vertices.Length != totalPoints)
        {
            vertices = new Vector3[totalPoints];
            colors = new Color[totalPoints];
            indices = new int[totalPoints];
        }

        int vertexIndex = 0;
        foreach (var scan in scanQueue)
        {
            float angle = scan.Angle_min;
            foreach (var range in scan.Ranges)
            {
                if (range >= scan.Range_min && range <= scan.Range_max)
                {
                    float x = Mathf.Cos(angle) * range * pointScale;
                    float y = 0;
                    float z = Mathf.Sin(angle) * range * pointScale;

                    vertices[vertexIndex] = new Vector3(x, y, z);
                    colors[vertexIndex] = Color.blue;
                    indices[vertexIndex] = vertexIndex;

                    vertexIndex++;
                }
                angle += scan.Angle_increment;
            }
        }

        UpdateMesh(vertexIndex);
    }

    private void UpdateMesh(int vertexCount)
    {
        pointCloudMesh.Clear();

        Array.Resize(ref vertices, vertexCount);
        Array.Resize(ref colors, vertexCount);
        Array.Resize(ref indices, vertexCount);

        pointCloudMesh.vertices = vertices;
        pointCloudMesh.colors = colors;
        pointCloudMesh.SetIndices(indices, MeshTopology.Points, 0);
    }

    void Update()
    {
        while (mainThreadActions.TryDequeue(out var action))
        {
            action?.Invoke();
        }
    }

    void OnDestroy()
    {
        if (scanSubscription != null)
        {
            ros2Node.RemoveSubscription<LaserScan>(scanSubscription);
        }
        if (ros2Node != null)
        {
            var ros2UnityComponent = FindObjectOfType<ROS2UnityComponent>();
            ros2UnityComponent.RemoveNode(ros2Node);
        }
    }
}