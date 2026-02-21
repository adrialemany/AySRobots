using UnityEngine;
using ROS2;

public class KeyboardRobot : MonoBehaviour
{
    public float moveSpeed = 5f;
    public float rotationSpeed = 100f;
    public float verticalSpeed = 3f;

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

    // --- REFERENCIAS AUTOMÁTICAS ---
    public RotateHelice leftHelice;
    public RotateHelice rightHelice;
    public Floater floaterScript; 

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null) rb = GetComponentInChildren<Rigidbody>();

        if (floaterScript == null)
        {
            floaterScript = GetComponent<Floater>();
            if (floaterScript == null) floaterScript = GetComponentInChildren<Floater>();
        }

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

        ros2Unity = FindObjectOfType<ROS2UnityComponent>();
        if (ros2Unity != null && ros2Unity.Ok())
        {
            string uniqueNodeName = "keyboard_robot_node_" + System.Guid.NewGuid().ToString().Substring(0, 5);
            ros2Node = ros2Unity.CreateNode(uniqueNodeName);
            
            twistPub = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>(topicName);
            twistMsg = new geometry_msgs.msg.Twist();
        }
    }

    void Update()
    {
        float horizontalInput = Input.GetAxis("Horizontal");
        float verticalInput = Input.GetAxis("Vertical");

        float altitudeInput = 0f;
        if (Input.GetKey(KeyCode.U)) altitudeInput = 1f;  
        else if (Input.GetKey(KeyCode.J)) altitudeInput = -1f; 

        // --- MOVIMIENTO FÍSICO HORIZONTAL EN UNITY ---
        Vector3 movement = transform.forward * verticalInput * moveSpeed * Time.deltaTime;
        transform.Translate(movement, Space.World);

        float rotation = horizontalInput * rotationSpeed * Time.deltaTime;
        transform.Rotate(0, rotation, 0);

        // --- MOVIMIENTO VERTICAL ---
        if (floaterScript != null)
        {
            floaterScript.currentDepthOffset -= altitudeInput * verticalSpeed * Time.deltaTime;
            floaterScript.currentDepthOffset = Mathf.Clamp(floaterScript.currentDepthOffset, 0f, 5.5f);
        }

        // --- PARED INVISIBLE (LÍMITES DE ALTURA) ---
        Vector3 pos = transform.position;
        if (pos.y > -0.421f || pos.y < -5.274f)
        {
            pos.y = Mathf.Clamp(pos.y, -5.274f, -0.421f);
            transform.position = pos;
            
            if (rb != null) rb.linearVelocity = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);
        }

        // --- CONTROL DIFERENCIAL DE LAS DOS HÉLICES ---
        float leftPower = verticalInput + horizontalInput;
        float rightPower = verticalInput - horizontalInput;

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

        // --- PUBLICAR EN ROS 2 ---
        if (twistPub != null && twistMsg != null)
        {
            twistMsg.Linear.X = verticalInput * moveSpeed;
            twistMsg.Linear.Z = altitudeInput * verticalSpeed; 
            twistMsg.Angular.Z = horizontalInput * rotationSpeed;
            
            twistPub.Publish(twistMsg);
        }
    }
}