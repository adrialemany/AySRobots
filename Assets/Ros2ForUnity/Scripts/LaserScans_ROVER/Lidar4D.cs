using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using System.Collections.Concurrent;
using sensor_msgs.msg;

public class Lidar4D : MonoBehaviour
{
    public string pointCloudTopicName = "/unilidar/cloud";
    public Material pointMaterial;
    public int messageBufferSize = 10; // Number of messages to accumulate
    public float pointScale = 100.0f; // Scaling factor for the points

    private ConcurrentQueue<Action> mainThreadActions = new ConcurrentQueue<Action>();
    private ROS2Node ros2Node;
    private Subscription<PointCloud2> pointCloudSubscription;
    private Mesh pointCloudMesh;

    private Queue<PointCloud2> pointCloudQueue = new Queue<PointCloud2>();
    private Vector3[] vertices;
    private Color[] colors;
    private int[] indices;

    void Start()
    {
        var ros2UnityComponent = FindObjectOfType<ROS2UnityComponent>();
        ros2Node = ros2UnityComponent.CreateNode("lidar4d_subscriber");

        pointCloudSubscription = ros2Node.CreateSubscription<PointCloud2>(pointCloudTopicName, PointCloudCallback);

        pointCloudMesh = new Mesh();
        GetComponent<MeshFilter>().mesh = pointCloudMesh;
    }

    private void PointCloudCallback(PointCloud2 pointCloud)
    {
        // Add the new message to the queue
        mainThreadActions.Enqueue(() =>
        {
            if (pointCloudQueue.Count >= messageBufferSize)
            {
                // If the queue is full, remove the oldest message
                pointCloudQueue.Dequeue();
            }

            pointCloudQueue.Enqueue(pointCloud); // Add the new message
            ProcessPointCloudQueue(); // Process the accumulated queue
        });
    }

    private void ProcessPointCloudQueue()
    {
        // Count all points in the queue
        int totalPoints = 0;
        foreach (var cloud in pointCloudQueue)
        {
            totalPoints += (int)(cloud.Row_step / cloud.Point_step);
        }

        // Ensure the buffers are large enough
        if (vertices == null || vertices.Length != totalPoints)
        {
            vertices = new Vector3[totalPoints];
            colors = new Color[totalPoints];
            indices = new int[totalPoints];
        }

        // Process each point cloud in the queue
        int vertexIndex = 0;
        foreach (var cloud in pointCloudQueue)
        {
            int pointCount = (int)(cloud.Row_step / cloud.Point_step);
            byte[] data = cloud.Data;
            int offset = 0;

            for (int i = 0; i < pointCount; i++)
            {
                float x = BitConverter.ToSingle(data, offset) * pointScale;
                float y = BitConverter.ToSingle(data, offset + 4) * pointScale;
                float z = BitConverter.ToSingle(data, offset + 8) * pointScale;

                vertices[vertexIndex] = new Vector3(x, y, z);
                colors[vertexIndex] = GetColorByDistance(vertices[vertexIndex].magnitude);
                indices[vertexIndex] = vertexIndex;

                vertexIndex++;
                offset += (int)cloud.Point_step;
            }
        }

        // Update the mesh with the accumulated points
        UpdateMesh(vertexIndex);
    }

    private void UpdateMesh(int vertexCount)
    {
        pointCloudMesh.Clear();

        // Resize the buffers to the actual number of points
        Array.Resize(ref vertices, vertexCount);
        Array.Resize(ref colors, vertexCount);
        Array.Resize(ref indices, vertexCount);

        pointCloudMesh.vertices = vertices;
        pointCloudMesh.colors = colors;
        pointCloudMesh.SetIndices(indices, MeshTopology.Points, 0);
    }

    private Color GetColorByDistance(float distance)
    {
        // Define the minimum and maximum distances for interpolation
        float minDistance = 0f; // Minimum distance
        float maxDistance = 4f; // Maximum distance, adjust as needed

        // Calculate the interpolation value 't' based on the distance
        float t = Mathf.InverseLerp(minDistance, maxDistance, distance);

        // Interpolate between red (near) and blue (far) based on 't'
        return Color.Lerp(Color.red, Color.blue, t);
    }

    void OnDestroy()
    {
        if (pointCloudSubscription != null)
        {
            ros2Node.RemoveSubscription<PointCloud2>(pointCloudSubscription);
        }
        if (ros2Node != null)
        {
            var ros2UnityComponent = FindObjectOfType<ROS2UnityComponent>();
            ros2UnityComponent.RemoveNode(ros2Node);
        }
    }

    void Update()
    {
        while (mainThreadActions.TryDequeue(out var action))
        {
            action?.Invoke();
        }
    }
}