using UnityEngine;
using ROS2;
using System;

[RequireComponent(typeof(Camera))]
public class CameraSensorROS2 : MonoBehaviour
{
    [Header("Configuración de ROS 2")]
    public string topicName = "/camera/image/compressed";
    public string frameId = "camera_link";
    
    [Header("Configuración de la Cámara")]
    public float publishRateHz = 15f; // 15 FPS es un buen equilibrio para simuladores
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;
    [Range(10, 100)] public int jpegQuality = 75; // Calidad de compresión

    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<sensor_msgs.msg.CompressedImage> imagePub;
    private sensor_msgs.msg.CompressedImage msg;

    private Camera captureCamera;
    private Texture2D texture2D;
    private Rect rect;
    private float timer = 0f;

    void Start()
    {
        // 1. Configurar la cámara y texturas
        captureCamera = GetComponent<Camera>();
        
        // Creamos la textura donde guardaremos los píxeles (RGB24 para color estándar)
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);

        // 2. Configurar ROS 2
        ros2Unity = FindObjectOfType<ROS2UnityComponent>();
        if (ros2Unity != null && ros2Unity.Ok())
        {
            string uniqueNodeName = "camera_node_" + Guid.NewGuid().ToString().Substring(0, 5);
            ros2Node = ros2Unity.CreateNode(uniqueNodeName);
            imagePub = ros2Node.CreatePublisher<sensor_msgs.msg.CompressedImage>(topicName);

            // Preparar la estructura estática del mensaje
            msg = new sensor_msgs.msg.CompressedImage();
            msg.Header = new std_msgs.msg.Header();
            msg.Header.Frame_id = frameId;
            msg.Format = "jpeg";

            Debug.Log($"Cámara inicializada publicando en: {topicName}");
        }
    }

    void Update()
    {
        if (imagePub == null) return;

        timer += Time.deltaTime;
        float publishInterval = 1f / publishRateHz;

        if (timer >= publishInterval)
        {
            PublishImage();
            timer = 0f;
        }
    }

    private void PublishImage()
    {
        // 1. Crear una textura temporal en la memoria gráfica de la tarjeta de vídeo
        RenderTexture renderTexture = RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, 24);
        
        // 2. Apuntar la cámara a esa textura y forzarla a hacer una "foto"
        captureCamera.targetTexture = renderTexture;
        captureCamera.Render();

        // 3. Leer los píxeles de la tarjeta gráfica y pasarlos a la RAM de Unity (Texture2D)
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(rect, 0, 0);
        texture2D.Apply();

        // 4. Limpiar para no saturar la memoria (¡Muy importante!)
        captureCamera.targetTexture = null;
        RenderTexture.active = null;
        RenderTexture.ReleaseTemporary(renderTexture);

        // 5. Comprimir la imagen a formato JPG
        byte[] imageData = texture2D.EncodeToJPG(jpegQuality);

        // 6. Asignar los datos al mensaje de ROS 2 y publicar
        msg.Data = imageData;
        imagePub.Publish(msg);
    }
}