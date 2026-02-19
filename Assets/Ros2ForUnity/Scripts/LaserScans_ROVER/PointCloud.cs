using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using sensor_msgs.msg;

public class LaserScanToPointCloud : MonoBehaviour
{
    public string scanTopicName = "/scan";
    public string pointCloudTopicName = "/point_cloud";

    private ROS2Node ros2Node;
    private Subscription<LaserScan> scanSubscription;
    private Publisher<PointCloud2> pointCloudPublisher;

    void Start()
    {
        var ros2UnityComponent = FindObjectOfType<ROS2UnityComponent>();
        ros2Node = ros2UnityComponent.CreateNode("laser_scan_to_point_cloud");

        scanSubscription = ros2Node.CreateSubscription<LaserScan>(scanTopicName, ScanCallback);
        pointCloudPublisher = ros2Node.CreatePublisher<PointCloud2>(pointCloudTopicName);
    }

    private void ScanCallback(LaserScan scan)
    {
        PointCloud2 pointCloud = ConvertLaserScanToPointCloud(scan);
        pointCloudPublisher.Publish(pointCloud);
    }

    private PointCloud2 ConvertLaserScanToPointCloud(LaserScan scan)
    {
        PointCloud2 pointCloud = new PointCloud2();
        pointCloud.Header = scan.Header;
        pointCloud.Height = 1;
        pointCloud.Width = (uint)scan.Ranges.Length;
        pointCloud.Is_bigendian = false;
        pointCloud.Point_step = 12; // 3 * 4 bytes (float32)
        pointCloud.Row_step = pointCloud.Point_step * pointCloud.Width;
        pointCloud.Is_dense = true;

        // Define the fields
        pointCloud.Fields = new PointField[]
        {
            new PointField { Name = "x", Offset = 0, Datatype = 7, Count = 1 },
            new PointField { Name = "y", Offset = 4, Datatype = 7, Count = 1 },
            new PointField { Name = "z", Offset = 8, Datatype = 7, Count = 1 }
        };

        List<byte> data = new List<byte>();
        float angle = scan.Angle_min;

        foreach (var range in scan.Ranges)
        {
            if (range >= scan.Range_min && range <= scan.Range_max)
            {
                float x = range * Mathf.Cos(angle);
                float y = range * Mathf.Sin(angle);
                float z = 0;

                data.AddRange(BitConverter.GetBytes(x));
                data.AddRange(BitConverter.GetBytes(y));
                data.AddRange(BitConverter.GetBytes(z));
            }
            angle += scan.Angle_increment;
        }

        // Ensure the data array size matches the expected size
        while (data.Count < pointCloud.Row_step * pointCloud.Height)
        {
            data.Add(0);
        }

        pointCloud.Data = data.ToArray();
        return pointCloud;
    }

    void OnDestroy()
    {
        if (scanSubscription != null)
        {
            ros2Node.RemoveSubscription<LaserScan>(scanSubscription);
        }
        if (pointCloudPublisher != null)
        {
            ros2Node.RemovePublisher<PointCloud2>(pointCloudPublisher);
        }
        if (ros2Node != null)
        {
            var ros2UnityComponent = FindObjectOfType<ROS2UnityComponent>();
            ros2UnityComponent.RemoveNode(ros2Node);
        }
    }
}