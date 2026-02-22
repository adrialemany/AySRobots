using System.Collections;
using UnityEngine;
using System.Runtime.InteropServices;
using ROS2;

public class RobotControllerButtons : MonoBehaviour
{
    [DllImport("user32.dll")]
    private static extern bool ShowWindow(System.IntPtr hWnd, int nCmdShow);
    
    [DllImport("user32.dll")]
    private static extern System.IntPtr GetActiveWindow();

    public float moveSpeed = 5f;
    public float rotationSpeed = 100f;
    public float verticalSpeed = 3f; 
    public float smoothTransition = 2f;
    public float stepChange = 2f;

    private float currentMoveSpeed = 0f;
    private float currentRotationSpeed = 0f;
    private float currentVerticalSpeed = 0f; 

    private float targetMoveSpeed = 0f;
    private float targetRotationSpeed = 0f;
    private float targetVerticalSpeed = 0f;  

    private bool isChangingMoveDirection = false;
    private bool isChangingRotationDirection = false;

    // --- VARIABLES DE ROS 2 ---
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Twist> twistPub;
    public string topicName = "/cmd_vel";
    private geometry_msgs.msg.Twist twistMsg;

    private Rigidbody rb;

    // --- REFERENCIAS AUTOMÁTICAS ---
    public Floater floaterScript; 
    public RotateHelice leftHelice;
    public RotateHelice rightHelice;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null) rb = GetComponentInChildren<Rigidbody>();

        if (floaterScript == null)
        {
            floaterScript = GetComponent<Floater>();
            if (floaterScript == null) floaterScript = GetComponentInChildren<Floater>();
        }
        if (floaterScript != null) floaterScript.enabled = true;

        if (leftHelice == null || rightHelice == null)
        {
            RotateHelice[] helices = GetComponentsInChildren<RotateHelice>();
            foreach (RotateHelice h in helices)
            {
                Vector3 localPos = transform.InverseTransformPoint(h.transform.position);
                if (localPos.x < 0) leftHelice = h; 
                else rightHelice = h;               
            }
        }

        ros2Unity = FindObjectOfType<ROS2UnityComponent>();
        if (ros2Unity != null && ros2Unity.Ok())
        {
            string uniqueNodeName = "robot_buttons_node_" + System.Guid.NewGuid().ToString().Substring(0, 5);
            ros2Node = ros2Unity.CreateNode(uniqueNodeName);
            
            twistPub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>(topicName);
            twistMsg = new geometry_msgs.msg.Twist();
        }
    }

    void Update()
    {
        currentMoveSpeed = Mathf.Lerp(currentMoveSpeed, targetMoveSpeed, Time.deltaTime * smoothTransition);
        currentRotationSpeed = Mathf.Lerp(currentRotationSpeed, targetRotationSpeed, Time.deltaTime * smoothTransition);
        currentVerticalSpeed = Mathf.Lerp(currentVerticalSpeed, targetVerticalSpeed, Time.deltaTime * smoothTransition);
        
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

        if (floaterScript != null && Mathf.Abs(currentVerticalSpeed) > 0.01f)
        {
            floaterScript.currentDepthOffset -= currentVerticalSpeed * Time.deltaTime;
            // Limitamos el offset para que el muelle no acumule tensión infinita
            floaterScript.currentDepthOffset = Mathf.Clamp(floaterScript.currentDepthOffset, 0f, 5.5f);
        }

        // --- PARED INVISIBLE (LÍMITES DE ALTURA) ---
        Vector3 pos = transform.position;
        if (pos.y > -0.421f || pos.y < -5.274f)
        {
            // Bloqueamos la posición en los límites exactos
            pos.y = Mathf.Clamp(pos.y, -5.274f, -0.421f);
            transform.position = pos;
            
            // Matamos la inercia vertical física para que no rebote
            if (rb != null) rb.linearVelocity = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);
        }

        float normalizedRotation = currentRotationSpeed / rotationSpeed; 
        float normalizedMove = currentMoveSpeed / moveSpeed;

        float leftPower = normalizedMove + normalizedRotation;
        float rightPower = normalizedMove - normalizedRotation;

        if (leftHelice != null)
        {
            if (leftPower > 0.1f) leftHelice.RotateForward();
            else if (leftPower < -0.1f) leftHelice.RotateBackward();
            else leftHelice.StopRotation();
        }
        if (rightHelice != null)
        {
            if (rightPower > 0.1f) rightHelice.RotateForward();
            else if (rightPower < -0.1f) rightHelice.RotateBackward();
            else rightHelice.StopRotation();
        }

        PublishToROS();
    }

    // --- NUEVAS FUNCIONES PARA MANTENER PULSADO ---
    public void OnPressAscend() { targetVerticalSpeed = verticalSpeed; }
    public void OnReleaseAscend() { targetVerticalSpeed = 0f; }
    
    public void OnPressDescend() { targetVerticalSpeed = -verticalSpeed; }
    public void OnReleaseDescend() { targetVerticalSpeed = 0f; }
    // ----------------------------------------------

    public void MoveForward()
    {
        if (currentMoveSpeed < 0 && !isChangingMoveDirection) { targetMoveSpeed = 0f; isChangingMoveDirection = true; }
        else if (isChangingMoveDirection) { targetMoveSpeed = moveSpeed; isChangingMoveDirection = false; }
        else { targetMoveSpeed = Mathf.Min(targetMoveSpeed + stepChange, moveSpeed); }
    }

    public void MoveBackward()
    {
        if (currentMoveSpeed > 0 && !isChangingMoveDirection) { targetMoveSpeed = 0f; isChangingMoveDirection = true; }
        else if (isChangingMoveDirection) { targetMoveSpeed = -moveSpeed; isChangingMoveDirection = false; }
        else { targetMoveSpeed = Mathf.Max(targetMoveSpeed - stepChange, -moveSpeed); }
    }

    public void RotateRight()
    {
        if (currentRotationSpeed < 0 && !isChangingRotationDirection) { targetRotationSpeed = 0f; isChangingRotationDirection = true; }
        else if (isChangingRotationDirection) { targetRotationSpeed = rotationSpeed; isChangingRotationDirection = false; }
        else { targetRotationSpeed = Mathf.Min(targetRotationSpeed + stepChange, rotationSpeed); }
    }

    public void RotateLeft()
    {
        if (currentRotationSpeed > 0 && !isChangingRotationDirection) { targetRotationSpeed = 0f; isChangingRotationDirection = true; }
        else if (isChangingRotationDirection) { targetRotationSpeed = -rotationSpeed; isChangingRotationDirection = false; }
        else { targetRotationSpeed = Mathf.Max(targetRotationSpeed - stepChange, -rotationSpeed); }
    }

    public void StopMoving()
    {
        targetMoveSpeed = 0f; targetRotationSpeed = 0f; targetVerticalSpeed = 0f;
        isChangingMoveDirection = false; isChangingRotationDirection = false; 
    }

    public void StopRotating()
    {
        targetRotationSpeed = 0f; isChangingRotationDirection = false;
    }

    public void Relocate()
    {
        if (floaterScript != null)
        {
            floaterScript.enabled = false;
            // Eliminada la línea que reseteaba el currentDepthOffset para mantener el lastre actual
        }
        
        if (rb != null)
        {
            rb.isKinematic = true;
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            rb.Sleep();
        }
        
        float currentYRotation = transform.rotation.eulerAngles.y;
        Vector3 currentPosition = transform.position;
        
        // Mantenemos la coordenada 'y' actual en lugar de forzar -0.421f
        transform.position = new Vector3(currentPosition.x, currentPosition.y, currentPosition.z);
        transform.rotation = Quaternion.Euler(0, currentYRotation, 0);
        
        targetMoveSpeed = 0f; currentMoveSpeed = 0f; targetRotationSpeed = 0f; currentRotationSpeed = 0f;
        targetVerticalSpeed = 0f; currentVerticalSpeed = 0f;
        isChangingMoveDirection = false; isChangingRotationDirection = false; 
        
        if (rb != null) 
        { 
            rb.isKinematic = false; 
            rb.WakeUp(); 
        }
        
        if (floaterScript != null) 
        {
            floaterScript.enabled = true;
        }
    }

    private void PublishToROS()
    {
        if (twistPub == null || twistMsg == null) return; 

        twistMsg.Linear.X = currentMoveSpeed;
        twistMsg.Linear.Z = currentVerticalSpeed; 
        twistMsg.Angular.Z = currentRotationSpeed;
        
        twistPub.Publish(twistMsg);
    }

    public void CloseGame() { Application.Quit(); }
    public void MinimizeAndClose() { ShowWindow(GetActiveWindow(), 2); Application.Quit(); }
}