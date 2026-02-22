using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

public class LidarSensorROS2 : MonoBehaviour
{
    [Header("Configuración del LiDAR")]
    public string topicName = "/lidar/points";
    public string frameId = "lidar_link";
    public float publishRateHz = 10f; 
    public float maxRange = 50f;      

    [Header("Resolución 3D")]
    public int horizontalResolution = 360; 
    public int verticalLayers = 16;        
    public float minVerticalAngle = -15f;  
    public float maxVerticalAngle = 15f;   

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<sensor_msgs.msg.PointCloud2> pointCloudPub;
    private sensor_msgs.msg.PointCloud2 msg;

    private float timer = 0f;

    private struct Point4D
    {
        public float x, y, z, intensity;
    }

    void Start()
    {
        ros2Unity = FindObjectOfType<ROS2UnityComponent>();
        if (ros2Unity != null && ros2Unity.Ok())
        {
            string uniqueNodeName = "lidar_node_" + System.Guid.NewGuid().ToString().Substring(0, 5);
            ros2Node = ros2Unity.CreateNode(uniqueNodeName);
            pointCloudPub = ros2Node.CreatePublisher<sensor_msgs.msg.PointCloud2>(topicName);
            
            InitializeMessage();
            Debug.Log($"LiDAR 4D inicializado en el topic: {topicName}");
        }
    }

    void Update()
    {
        timer += Time.deltaTime;
        float publishInterval = 1f / publishRateHz;

        if (timer >= publishInterval)
        {
            ScanAndPublish();
            timer = 0f;
        }
    }

    private void ScanAndPublish()
    {
        if (pointCloudPub == null) return;

        List<Point4D> cloudPoints = new List<Point4D>();

        float verticalStep = (verticalLayers > 1) ? (maxVerticalAngle - minVerticalAngle) / (verticalLayers - 1) : 0;
        float horizontalStep = 360f / horizontalResolution;

        for (int i = 0; i < verticalLayers; i++)
        {
            float vAngle = minVerticalAngle + (i * verticalStep);
            Quaternion vRotation = Quaternion.Euler(-vAngle, 0, 0);

            for (int j = 0; j < horizontalResolution; j++)
            {
                float hAngle = j * horizontalStep;
                Quaternion hRotation = Quaternion.Euler(0, hAngle, 0);
                
                Vector3 rayDirection = hRotation * vRotation * Vector3.forward;
                Vector3 worldRayDirection = transform.rotation * rayDirection;

                if (Physics.Raycast(transform.position, worldRayDirection, out RaycastHit hit, maxRange))
                {
                    Vector3 localPoint = transform.InverseTransformPoint(hit.point);

                    float rosX = localPoint.z;
                    float rosY = -localPoint.x;
                    float rosZ = localPoint.y;

                    float intensity = 1.0f - (hit.distance / maxRange);

                    cloudPoints.Add(new Point4D { x = rosX, y = rosY, z = rosZ, intensity = intensity });
                }
            }
        }

        if (cloudPoints.Count > 0)
        {
            PackAndPublishCloud(cloudPoints);
        }
    }

    private void PackAndPublishCloud(List<Point4D> points)
    {
        // Propiedades con la primera letra en mayúscula según tu versión de ROS2ForUnity
        msg.Width = (uint)points.Count;
        msg.Row_step = msg.Point_step * msg.Width;

        byte[] byteArray = new byte[points.Count * msg.Point_step];
        int offset = 0;

        foreach (var p in points)
        {
            Buffer.BlockCopy(BitConverter.GetBytes(p.x), 0, byteArray, offset, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(p.y), 0, byteArray, offset + 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(p.z), 0, byteArray, offset + 8, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(p.intensity), 0, byteArray, offset + 12, 4);
            
            offset += (int)msg.Point_step;
        }

        msg.Data = byteArray;
        pointCloudPub.Publish(msg);
    }

    private void InitializeMessage()
    {
        msg = new sensor_msgs.msg.PointCloud2();
        msg.Header = new std_msgs.msg.Header();
        msg.Header.Frame_id = frameId;
        
        msg.Height = 1; 
        msg.Is_dense = true; 
        msg.Is_bigendian = false;
        msg.Point_step = 16; 

        msg.Fields = new sensor_msgs.msg.PointField[4];
        
        // Uso de Name, Offset, Datatype, Count en mayúscula y el valor 7 para FLOAT32
        msg.Fields[0] = new sensor_msgs.msg.PointField { Name = "x", Offset = 0, Datatype = 7, Count = 1 };
        msg.Fields[1] = new sensor_msgs.msg.PointField { Name = "y", Offset = 4, Datatype = 7, Count = 1 };
        msg.Fields[2] = new sensor_msgs.msg.PointField { Name = "z", Offset = 8, Datatype = 7, Count = 1 };
        msg.Fields[3] = new sensor_msgs.msg.PointField { Name = "intensity", Offset = 12, Datatype = 7, Count = 1 };
    }
}