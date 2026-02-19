using UnityEngine;
using ROS2; // Usamos el namespace de ROS2ForUnity

public class KeyboardRobot : MonoBehaviour
{
    public float moveSpeed = 5f;
    public float rotationSpeed = 100f;

    private KeyboardMovement keyboardMovementScript;
    private CameraSwitcher cameraSwitcherScript;

    public Canvas canvasToActivate;
    public Canvas canvasToDestroy;

    private Rigidbody rb;

    // --- VARIABLES DE ROS 2 ---
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Twist> twistPub;
    public string topicName = "/cmd_vel";
    private geometry_msgs.msg.Twist twistMsg;
    // --------------------------

    // --- REFERENCIA A LAS HÉLICES ---
    public RotateHelice rotateHeliceScript;
    // --------------------------------

    void Start()
    {
        // 1. Lógica original de desactivar otros scripts y Canvas
        keyboardMovementScript = FindObjectOfType<KeyboardMovement>();
        if (keyboardMovementScript != null)
        {
            keyboardMovementScript.enabled = false;
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }

        cameraSwitcherScript = FindObjectOfType<CameraSwitcher>();
        if (cameraSwitcherScript != null) cameraSwitcherScript.enabled = false;

        if (canvasToActivate != null) canvasToActivate.gameObject.SetActive(true);
        if (canvasToDestroy != null) Destroy(canvasToDestroy.gameObject);

        rb = GetComponent<Rigidbody>();
        if (rb == null) Debug.LogError("No se encontró un componente Rigidbody.");

        // 2. Inicialización de ROS 2
        ros2Unity = FindObjectOfType<ROS2UnityComponent>();
        if (ros2Unity != null && ros2Unity.Ok())
        {
            ros2Node = ros2Unity.CreateNode("keyboard_robot_node");
            twistPub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>(topicName);
            twistMsg = new geometry_msgs.msg.Twist();
        }
    }

    void Update()
    {
        float horizontalInput = Input.GetAxis("Horizontal");
        float verticalInput = Input.GetAxis("Vertical");

        // --- MOVIMIENTO FÍSICO EN UNITY ---
        Vector3 movement = transform.forward * verticalInput * moveSpeed * Time.deltaTime;
        transform.Translate(movement, Space.World);

        float rotation = horizontalInput * rotationSpeed * Time.deltaTime;
        transform.Rotate(0, rotation, 0);

        // --- CONTROL DE HÉLICES VISUALES ---
        if (rotateHeliceScript != null)
        {
            if (verticalInput > 0.1f) rotateHeliceScript.RotateForward();
            else if (verticalInput < -0.1f) rotateHeliceScript.RotateBackward();
            else rotateHeliceScript.StopRotation();
        }

        // --- PUBLICAR EN ROS 2 ---
        if (twistPub != null && twistMsg != null)
        {
            // Asignamos la velocidad proporcional al input
            twistMsg.Linear.X = verticalInput * moveSpeed;
            twistMsg.Angular.Z = horizontalInput * rotationSpeed; // En ROS, giro positivo es izquierda
            
            twistPub.Publish(twistMsg);
        }
    }
}