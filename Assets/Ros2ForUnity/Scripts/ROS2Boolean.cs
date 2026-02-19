using UnityEngine;

namespace ROS2
{
    public class ROS2Boolean : MonoBehaviour
    {
        // Variables privadas
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node, ros2NodeAvanzar, ros2NodeVueltas;
        private IPublisher<std_msgs.msg.Bool> bool_publisher;
        private IPublisher<std_msgs.msg.Bool> avanzar_publisher;
        private IPublisher<std_msgs.msg.Bool> vueltas_publisher;



        // Método Start
        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
        }

        // ======== AVANZAR RECTO ========
        public void Avanzar()
        {
            if (ros2Unity.Ok())
            {
                ros2Node = ros2Unity.CreateNode("NodoGeneral");
                avanzar_publisher = ros2NodeAvanzar.CreatePublisher<std_msgs.msg.Bool>("avanzar");

                std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
                msg.Data = true;
                avanzar_publisher.Publish(msg);
            }
        }

        // Activar a true
        public void AvanzarTrue()
        {
            if (ros2Unity.Ok())
            {
                if (ros2NodeAvanzar == null)
                {
                    ros2NodeAvanzar = ros2Unity.CreateNode("NodoAvanzarTrue");
                    avanzar_publisher = ros2NodeAvanzar.CreatePublisher<std_msgs.msg.Bool>("avanzar");
                }
                std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
                msg.Data = true;
                avanzar_publisher.Publish(msg);
            }
        }

        // Activar a false
        public void AvanzarFalse()
        {
            if (ros2Unity.Ok())
            {
                if (ros2NodeAvanzar == null)
                {
                    ros2NodeAvanzar = ros2Unity.CreateNode("NodoAvanzarFalse");
                    avanzar_publisher = ros2NodeAvanzar.CreatePublisher<std_msgs.msg.Bool>("avanzar");
                }
                std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
                msg.Data = false;
                avanzar_publisher.Publish(msg);
            }
        }

        // ======== DAR VUELTAS ========
        // Activar a true
        public void VueltasTrue()
        {
            if (ros2Unity.Ok())
            {
                if (ros2NodeVueltas == null)
                {
                    ros2NodeVueltas = ros2Unity.CreateNode("NodoVueltasTrue");
                    vueltas_publisher = ros2NodeVueltas.CreatePublisher<std_msgs.msg.Bool>("vueltas");
                }
                std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
                msg.Data = true;
                vueltas_publisher.Publish(msg);
            }
        }

        // Activar a false
        public void VueltasFalse()
        {
            if (ros2Unity.Ok())
            {
                if (ros2NodeVueltas == null)
                {
                    ros2NodeVueltas = ros2Unity.CreateNode("NodoVueltasFalse");
                    vueltas_publisher = ros2NodeVueltas.CreatePublisher<std_msgs.msg.Bool>("vueltas");
                }
                std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
                msg.Data = false;
                vueltas_publisher.Publish(msg);
            }
        }

        // ======== DESACTIVAR AMBOS ========
        public void DesactivarAmbos()
        {
            if (ros2Unity.Ok())
            {
                // Inicializar nodo y publicador si aún no se han creado
                if (ros2Node == null)
                {
                    ros2Node = ros2Unity.CreateNode("NodoGeneral");

                    // Crear publicadores para ambos tópicos
                    avanzar_publisher = ros2Node.CreatePublisher<std_msgs.msg.Bool>("avanzar");
                    vueltas_publisher = ros2Node.CreatePublisher<std_msgs.msg.Bool>("vueltas");
                }

                // Publicar False en 'avanzar'
                std_msgs.msg.Bool avanzar_msg = new std_msgs.msg.Bool();
                avanzar_msg.Data = false;
                avanzar_publisher.Publish(avanzar_msg);

                // Publicar False en 'vueltas'
                std_msgs.msg.Bool vueltas_msg = new std_msgs.msg.Bool();
                vueltas_msg.Data = false;
                vueltas_publisher.Publish(vueltas_msg);
            }
        }

    }
}
