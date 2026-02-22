using UnityEngine;
using ROS2;

public class RobotROS2Subscriber : MonoBehaviour
{
    [Header("Ajustes visuales de las Hélices")]
    public float maxMoveSpeedForHelices = 5f;
    public float maxRotSpeedForHelices = 100f;

    // --- VARIABLES DE ROS 2 ---
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private ISubscription<geometry_msgs.msg.Twist> twistSub;
    public string topicName = "/cmd_vel";

    // --- VELOCIDADES RECIBIDAS DESDE ROS 2 ---
    private float currentLinearX = 0f;
    private float currentLinearZ = 0f;
    private float currentAngularZ = 0f;

    private Rigidbody rb;

    // --- REFERENCIAS AUTOMÁTICAS (AHORA PRIVADAS PARA OCULTARLAS DEL INSPECTOR) ---
    private Floater floaterScript; 
    private RotateHelice leftHelice;
    private RotateHelice rightHelice;

    void Start()
    {
        // --- 1. AUTO-BÚSQUEDA DE COMPONENTES DEL ROBOT ---
        rb = GetComponent<Rigidbody>();
        if (rb == null) rb = GetComponentInChildren<Rigidbody>();

        // Busca el Floater automáticamente
        floaterScript = GetComponent<Floater>();
        if (floaterScript == null) floaterScript = GetComponentInChildren<Floater>();
        
        if (floaterScript != null) floaterScript.enabled = true;

        // Busca y asigna las hélices izquierda y derecha por su posición 3D
        RotateHelice[] helices = GetComponentsInChildren<RotateHelice>();
        foreach (RotateHelice h in helices)
        {
            Vector3 localPos = transform.InverseTransformPoint(h.transform.position);
            if (localPos.x < 0) leftHelice = h; 
            else rightHelice = h;               
        }

        // --- 2. CONFIGURACIÓN DEL SUSCRIPTOR DE ROS 2 ---
        // Busca el componente principal de ROS2 de forma automática
        ros2Unity = FindObjectOfType<ROS2UnityComponent>();
        if (ros2Unity != null && ros2Unity.Ok())
        {
            string uniqueNodeName = "robot_subscriber_node_" + System.Guid.NewGuid().ToString().Substring(0, 5);
            ros2Node = ros2Unity.CreateNode(uniqueNodeName);
            
            // Nos suscribimos al topic. Cuando llegue un mensaje, extraemos los datos.
            twistSub = ros2Node.CreateSubscription<geometry_msgs.msg.Twist>(
                topicName, 
                msg => 
                {
                    currentLinearX = (float)msg.Linear.X;
                    currentLinearZ = (float)msg.Linear.Z;
                    currentAngularZ = (float)msg.Angular.Z;
                });
                
            Debug.Log("Suscrito automáticamente al topic: " + topicName);
        }
        else
        {
            Debug.LogError("No se encontró el ROS2UnityComponent en la escena.");
        }
    }

    void Update()
    {
        // --- 1. MOVIMIENTO HORIZONTAL Y GIRO ---
        if (Mathf.Abs(currentLinearX) > 0.01f)
        {
            Vector3 movement = transform.forward * currentLinearX * Time.deltaTime;
            transform.Translate(movement, Space.World);
        }

        if (Mathf.Abs(currentAngularZ) > 0.01f)
        {
            float rotation = currentAngularZ * Time.deltaTime;
            transform.Rotate(0, rotation, 0);
        }

        // --- 2. MOVIMIENTO VERTICAL (SISTEMA DE LASTRE) ---
        if (floaterScript != null && Mathf.Abs(currentLinearZ) > 0.01f)
        {
            floaterScript.currentDepthOffset -= currentLinearZ * Time.deltaTime;
            floaterScript.currentDepthOffset = Mathf.Clamp(floaterScript.currentDepthOffset, 0f, 5.5f);
        }

        // --- 3. PAREDES INVISIBLES (LÍMITES DE ALTURA) ---
        Vector3 pos = transform.position;
        if (pos.y > -0.421f || pos.y < -5.274f)
        {
            pos.y = Mathf.Clamp(pos.y, -5.274f, -0.421f);
            transform.position = pos;
            
            if (rb != null) rb.linearVelocity = new Vector3(rb.linearVelocity.x, 0f, rb.linearVelocity.z);
        }

        // --- 4. CONTROL VISUAL DE LAS HÉLICES ---
        float normalizedMove = currentLinearX / maxMoveSpeedForHelices;
        float normalizedRotation = currentAngularZ / maxRotSpeedForHelices; 

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
    }
}