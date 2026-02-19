using System.Collections;
using UnityEngine;
using System.Runtime.InteropServices;
using ROS2; // Usamos el namespace de ROS2ForUnity

public class RobotControllerButtons : MonoBehaviour
{
    // Respetamos tu código original de Windows
    [DllImport("user32.dll")]
    private static extern bool ShowWindow(System.IntPtr hWnd, int nCmdShow);
    [DllImport("user32.dll")]
    private static extern System.IntPtr GetActiveWindow();

    public float moveSpeed = 5f;
    public float rotationSpeed = 100f;
    public float smoothTransition = 2f;
    public float stepChange = 2f;

    private float currentMoveSpeed = 0f;
    private float currentRotationSpeed = 0f;

    private float targetMoveSpeed = 0f;
    private float targetRotationSpeed = 0f;

    private bool isChangingMoveDirection = false;
    private bool isChangingRotationDirection = false;

    // --- VARIABLES DE ROS 2 ---
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Twist> twistPub;
    public string topicName = "/cmd_vel";
    private geometry_msgs.msg.Twist twistMsg;
    // --------------------------

    public MonoBehaviour floaterScript;
    private Rigidbody rb;

    void Start()
    {
        // 1. Buscamos el componente principal de ROS 2 en la escena
        ros2Unity = FindObjectOfType<ROS2UnityComponent>();
        
        if (ros2Unity != null && ros2Unity.Ok())
        {
            // 2. Creamos el nodo y el publicador
            ros2Node = ros2Unity.CreateNode("robot_buttons_node");
            twistPub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>(topicName);
            twistMsg = new geometry_msgs.msg.Twist();
            Debug.Log("Conexión ROS 2 inicializada y publicador registrado.");
        }
        else
        {
            Debug.LogError("No se encontró ROS2UnityComponent en la escena o no está inicializado.");
        }

        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            Debug.LogError("No se encontró un componente Rigidbody en el objeto.");
        }
        if (floaterScript != null)
        {
            floaterScript.enabled = true;
            Debug.Log("Script Floater activado.");
        }
    }

    void Update()
    {
        currentMoveSpeed = Mathf.Lerp(currentMoveSpeed, targetMoveSpeed, Time.deltaTime * smoothTransition);
        currentRotationSpeed = Mathf.Lerp(currentRotationSpeed, targetRotationSpeed, Time.deltaTime * smoothTransition);
        
        if (Mathf.Abs(currentMoveSpeed) > 0.01f)
        {
            Vector3 movement = transform.forward * currentMoveSpeed * Time.deltaTime;
            transform.Translate(movement, Space.World);
        }
        if (Mathf.Abs(currentRotationSpeed) > 0.01f)
        {
            float rotation = currentRotationSpeed * Time.deltaTime;
            transform.Rotate(0, rotation, 0);
        }
    }

    public void MoveForward()
    {
        if (currentMoveSpeed < 0 && !isChangingMoveDirection) { targetMoveSpeed = 0f; isChangingMoveDirection = true; }
        else if (isChangingMoveDirection) { targetMoveSpeed = moveSpeed; isChangingMoveDirection = false; }
        else { targetMoveSpeed = Mathf.Min(targetMoveSpeed + stepChange, moveSpeed); }
        PublishToROS();
    }

    public void MoveBackward()
    {
        if (currentMoveSpeed > 0 && !isChangingMoveDirection) { targetMoveSpeed = 0f; isChangingMoveDirection = true; }
        else if (isChangingMoveDirection) { targetMoveSpeed = -moveSpeed; isChangingMoveDirection = false; }
        else { targetMoveSpeed = Mathf.Max(targetMoveSpeed - stepChange, -moveSpeed); }
        PublishToROS();
    }

    public void RotateRight()
    {
        if (currentRotationSpeed < 0 && !isChangingRotationDirection) { targetRotationSpeed = 0f; isChangingRotationDirection = true; }
        else if (isChangingRotationDirection) { targetRotationSpeed = rotationSpeed; isChangingRotationDirection = false; }
        else { targetRotationSpeed = Mathf.Min(targetRotationSpeed + stepChange, rotationSpeed); }
        PublishToROS();
    }

    public void RotateLeft()
    {
        if (currentRotationSpeed > 0 && !isChangingRotationDirection) { targetRotationSpeed = 0f; isChangingRotationDirection = true; }
        else if (isChangingRotationDirection) { targetRotationSpeed = -rotationSpeed; isChangingRotationDirection = false; }
        else { targetRotationSpeed = Mathf.Max(targetRotationSpeed - stepChange, -rotationSpeed); }
        PublishToROS();
    }

    public void StopMoving()
    {
        targetMoveSpeed = 0f; currentMoveSpeed = 0f; targetRotationSpeed = 0f; currentRotationSpeed = 0f;
        isChangingMoveDirection = false; isChangingRotationDirection = false;
        PublishToROS();
    }

    public void StopRotating()
    {
        targetRotationSpeed = 0f; isChangingRotationDirection = false;
    }

    public void Relocate()
    {
        if (floaterScript != null) floaterScript.enabled = false;
        rb.isKinematic = true;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.Sleep();
        
        float currentYRotation = transform.rotation.eulerAngles.y;
        Vector3 currentPosition = transform.position;
        transform.position = new Vector3(currentPosition.x, -0.421f, currentPosition.z);
        transform.rotation = Quaternion.Euler(0, currentYRotation, 0);
        
        targetMoveSpeed = 0f; currentMoveSpeed = 0f; targetRotationSpeed = 0f; currentRotationSpeed = 0f;
        isChangingMoveDirection = false; isChangingRotationDirection = false;
        
        if (rb != null) { rb.isKinematic = false; rb.WakeUp(); }
        if (floaterScript != null) floaterScript.enabled = true;
    }

    private void PublishToROS()
    {
        if (twistPub == null || twistMsg == null) return; // Evita errores si ROS no está listo

        // En ROS 2, los tipos base van en mayúscula (Linear.X, Angular.Z)
        twistMsg.Linear.X = currentMoveSpeed;
        twistMsg.Angular.Z = currentRotationSpeed;
        
        twistPub.Publish(twistMsg);
    }

    public void CloseGame() { Application.Quit(); }
    public void MinimizeAndClose() { ShowWindow(GetActiveWindow(), 2); Application.Quit(); }
}