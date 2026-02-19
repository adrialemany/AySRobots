// RosTopics.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
  Esta clase contiene todas las rutas de topics y variables compartidas entre scripts.
  Al ser estática, no necesitas instanciarla, solo llamarla directamente.
*/
public static class RosTopics
{
    //[Header("ROS2 Configuration")]
    public const string topic_LogitechCamRight = "/gui/logitech_camera_right";
    public const string topic_LogitechCamLeft  = "/gui/logitech_camera_left";
    public const string topic_SiyiCamera       = "/gui/siyi_camera";
    public const string topic_ZedCamera        = "/gui/zed2_camera";
    public const string topic_RealsenseCamera  = "/gui/realsense_camera";

    public const string topic_Lidar2D          = "/gui/lidar_2d";
    public const string topic_Lidar4D          = "/gui/lidar_4d";

    public const string topic_NCams            = "/n_cams";
    public const string topic_Ruedas           = "/gui/wheels";
    public const string topic_Shutdown         = "/gui/shutdown";

    //[Header("Image Topics")]
    public const string imageTopic_LogitechCamRight = "/logitech_camera_right/image_raw/compressed";
    public const string imageTopic_LogitechCamLeft  = "/logitech_camera_left/image_raw/compressed";
    public const string imageTopic_SiyiCamera       = "/siyi_camera/image_raw/compressed";
    public const string imageTopic_ZedCamera        = "/zed/zed_node/stereo_raw/image_raw_color/compressed";
    //
    // public const string imageTopic_ZedCamera_stereo = "/zed/zed_node/stereo_raw/image_raw_color/compressed";
    // public const string imageTopic_ZedCamera_right  = "/zed/zed_node/right_raw/image_raw_color/compressed";
    // public const string imageTopic_ZedCamera_left   = "/zed/zed_node/left_raw/image_raw_color/compressed";
    //
    public const string imageTopic_RealsenseCamera  = "/realsense_camera/image_raw/compressed";

    //[Header("Lidar Topics")]
    //public const string lidarTopic_Lidar2D = "/scan";
    //public const string lidarTopic_Lidar4D = "/unilidar/cloud";
    
    // Puedes agregar aquí más topics o variables comunes
}
