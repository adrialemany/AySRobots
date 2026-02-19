using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace ROS2
{

public class ROS2Teleop : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Twist> cmd_vel_publisher;

    public GameObject MainCanvas;
    public GameObject CanvasTeleop;
    
    public Button S_Button, W_Button, A_Button, D_Button, STOP_Button;
    //public Button ESC_Button;
    private Color pressedColor = new Color32(255, 255, 255, 255); // Color FFFFFF
    private Color originalColor = new Color32(200, 200, 200, 255); // Color C8C8C8
    private Color redColor = new Color32(255, 153, 153, 255); // Color FF9999
    

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    // ######### TECLAS #########
    
    public void ActivatedKey_W()
    {
        //Debug.Log("PULSANDO W");
        changeColor(W_Button, pressedColor);
    }

    public void ActivatedKey_S()
    {
        //Debug.Log("PULSANDO S");
        changeColor(S_Button, pressedColor);
    }

    public void ActivatedKey_A()
    {
        //Debug.Log("PULSANDO A");
        changeColor(A_Button, pressedColor);
    }

    public void ActivatedKey_D()
    {
        //Debug.Log("PULSANDO D");
        changeColor(D_Button, pressedColor);
    }

    public void ActivatedKey_STOP()
    {
        //Debug.Log("PULSANDO STOP");
        changeColor(STOP_Button, redColor);
    }
    /*
    public void ActivatedKey_ESC()
    {
        Debug.Log("PULSANDO ESC");
        changeColor(ESC_Button, pressedColor);
        MoveWithCMDVel(0, 0);
        MainCanvas.SetActive(true);
        CanvasTeleop.SetActive(false);
    }
    */
    // ###########################

    // ########## ROS2 ##########
    public void MoveWithCMDVel(float vel_x, float ang_z)
    {
        if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("NodoTeleop");
                cmd_vel_publisher = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>("cmd_vel");
            }

            geometry_msgs.msg.Twist msg = new geometry_msgs.msg.Twist();
            msg.Linear.X = vel_x;
            msg.Angular.Z = ang_z;
            cmd_vel_publisher.Publish(msg);
        }
    }


    void Update()
    {
        float verticalInput = Input.GetAxis("Vertical");
        float horizontalInput = Input.GetAxis("Horizontal");
        float stopInput = Input.GetAxis("Jump");
        float escInput = Input.GetAxis("Escape");

        // =========== VERTICAL ===========
        
        if (verticalInput > 0) {
            // Lógica para cuando se presiona "W"
            ActivatedKey_W();
        }
        else if (verticalInput < 0) {
            // Lógica para cuando se presiona "S"
            ActivatedKey_S();
        }
        else {
            changeColor(S_Button, originalColor);
            changeColor(W_Button, originalColor);
        }

        // =========== HORIZONTAL ===========
        if (horizontalInput > 0) {
            // Lógica para cuando se presiona "D"
            ActivatedKey_D();
        }
        else if (horizontalInput < 0) {
            // Lógica para cuando se presiona "A"
            ActivatedKey_A();
        }
        else {
            changeColor(A_Button, originalColor);
            changeColor(D_Button, originalColor);
        }

        // =========== STOP ===========
        if (stopInput > 0) {
            // Lógica para cuando se presiona "SPACE"
            ActivatedKey_STOP();
        }
        else {
            changeColor(STOP_Button, originalColor);
        }
        /*
        // =========== ESC ===========
        if (escInput > 0) {
            // Lógica para cuando se presiona "ESC"
            ActivatedKey_ESC();
            verticalInput = 0;
            horizontalInput = 0;
        }
        else {
            changeColor(ESC_Button, originalColor);
        }
        */
        //Debug.Log("Velocidad delante: " + verticalInput);
        //Debug.Log("Velocidad lados: " + horizontalInput);

        if (verticalInput >= 0)
            MoveWithCMDVel(verticalInput, -horizontalInput);
        else
            MoveWithCMDVel(verticalInput, horizontalInput);       
    }
    
    public void changeColor(Button button, Color color)
    {
        button.GetComponent<Image>().color = color;
    }
    
}

}  // namespace ROS2