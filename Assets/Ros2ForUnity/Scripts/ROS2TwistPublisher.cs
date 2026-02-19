using UnityEngine;
using ROS2;

public class Ros2TwistPublisher : MonoBehaviour
{
    // Referencia al componente principal que conecta con ROS
    private ROS2UnityComponent ros2Unity;
    // El nodo que crearemos dentro de Unity
    private ROS2Node ros2Node;
    // El publicador que enviará los mensajes de velocidad
    private IPublisher<geometry_msgs.msg.Twist> twistPub;

    void Start()
    {
        // Buscamos el componente ROS2UnityComponent en el mismo objeto
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    void Update()
    {
        // 1. Comprobamos si el sistema ROS está listo
        if (ros2Unity.Ok())
        {
            // 2. Si el nodo no existe aún, lo creamos (solo una vez)
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("unity_cmd_vel_publisher");
                twistPub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>("/cmd_vel");
            }

            // 3. Creamos el mensaje
            geometry_msgs.msg.Twist msg = new geometry_msgs.msg.Twist();
            
            // Vamos a hacer que se mueva en círculo:
            msg.Linear.X = 2.0;  // 2 m/s hacia adelante
            msg.Angular.Z = 1.0; // 1 rad/s girando

            // 4. Publicamos el mensaje
            twistPub.Publish(msg);
        }
    }
}