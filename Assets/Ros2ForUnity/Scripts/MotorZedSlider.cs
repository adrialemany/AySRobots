using UnityEngine;
using UnityEngine.UI;

namespace ROS2
{
    public class MotorZedSlider : MonoBehaviour
    {
        // Variables privadas
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;

        public Slider SliderX;
        public Slider SliderY;

        // zed2_rotation_horizontal

        private IPublisher<std_msgs.msg.Int32> SliderXPublisher;
        private IPublisher<std_msgs.msg.Int32> SliderYPublisher;

        // Método Start
        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
        }

        void Update()
        {
            // Crear el nodo y publicador si aún no se han creado
            if (ros2Node == null && ros2Unity.Ok())
            {
                ros2Node = ros2Unity.CreateNode("NodoMotoresZed");
                SliderXPublisher = ros2Node.CreatePublisher<std_msgs.msg.Int32>("zed2_rotation_horizontal");
                SliderYPublisher = ros2Node.CreatePublisher<std_msgs.msg.Int32>("zed2_rotation_vertical");
            }
        } 

        public void SliderXPublisherMethod()
        {
            if (SliderXPublisher != null)
            {
                std_msgs.msg.Int32 msg = new std_msgs.msg.Int32();
                msg.Data = (int)SliderX.value;
                SliderXPublisher.Publish(msg);
            }
        }

        public void SliderYPublisherMethod()
        {
            if (SliderYPublisher != null)
            {
                std_msgs.msg.Int32 msg = new std_msgs.msg.Int32();
                msg.Data = (int)SliderY.value;
                SliderYPublisher.Publish(msg);
            }
        }

    }
}