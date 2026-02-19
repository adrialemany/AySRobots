using System.Collections.Generic;
using UnityEngine;
using ROS2;
using sensor_msgs.msg;

public class PointCloudVisualizer : MonoBehaviour
{
    private PointCloud2 currentCloud;
    private ROS2Node ros2Node;
    private Subscription<PointCloud2> cloudSubscription;

    private Mesh mesh; // Malla para los puntos
    private List<Vector3> points = new List<Vector3>();
    private List<Color> colors = new List<Color>();

    public string topic = "/unilidar/cloud";
    public float minDistance = 0.0f; // Distancia mínima
    public float maxDistance = 10.0f; // Ajustar al rango de tu Lidar

    void Start()
    {
        // Inicializar ROS2
        var ros2UnityComponent = FindObjectOfType<ROS2UnityComponent>();
        ros2Node = ros2UnityComponent.CreateNode("point_cloud_subscriber");
        cloudSubscription = ros2Node.CreateSubscription<PointCloud2>(topic, PointCloudCallback);

        // Inicializar la malla
        mesh = new Mesh();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        GetComponent<MeshFilter>().mesh = mesh;
    }

    private void PointCloudCallback(PointCloud2 cloud)
    {
        currentCloud = cloud;
    }

    void Update()
    {
        if (currentCloud != null)
        {
            ParseAndVisualizeCloud();
        }
    }

    private void ParseAndVisualizeCloud()
    {
        points.Clear();
        colors.Clear();

        byte[] data = currentCloud.Data;
        int pointStep = (int)currentCloud.Point_step;
        int numPoints = data.Length / pointStep;

        for (int i = 0; i < numPoints; i++)
        {
            int offset = i * pointStep;

            // Extraer X, Y, Z
            float x = System.BitConverter.ToSingle(data, offset + 0);
            float y = System.BitConverter.ToSingle(data, offset + 4);
            float z = System.BitConverter.ToSingle(data, offset + 8);

            float distance = Mathf.Sqrt(x * x + y * y + z * z);

            if (!float.IsNaN(x) && !float.IsNaN(y) && !float.IsNaN(z))
            {
                points.Add(new Vector3(x, y, z));

                // Calcular color según la distancia
                float t = Mathf.InverseLerp(minDistance, maxDistance, distance);
                Color pointColor = Color.Lerp(Color.red, Color.magenta, t);
                colors.Add(pointColor);
            }
        }

        UpdateMesh();
    }

    private void UpdateMesh()
    {
        mesh.Clear();

        // Asignar vértices y colores
        mesh.SetVertices(points);
        mesh.SetColors(colors);

        // Crear índices para cada punto (draw as points)
        int[] indices = new int[points.Count];
        for (int i = 0; i < indices.Length; i++)
        {
            indices[i] = i;
        }

        mesh.SetIndices(indices, MeshTopology.Points, 0);
        mesh.RecalculateBounds();
    }

    void OnDestroy()
    {
        if (cloudSubscription != null)
        {
            ros2Node.RemoveSubscription<PointCloud2>(cloudSubscription);
        }
        if (ros2Node != null)
        {
            var ros2UnityComponent = FindObjectOfType<ROS2UnityComponent>();
            ros2UnityComponent.RemoveNode(ros2Node);
        }
    }
}
