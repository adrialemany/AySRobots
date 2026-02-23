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
    private ISubscription<std_msgs.msg.Bool> relocateSub; // Nova subscripció
    
    public string topicName = "/cmd_vel";
    public string relocateTopicName = "/relocate"; // Nou tòpic

    // --- VELOCIDADES RECIBIDAS DESDE ROS 2 ---
    private float currentLinearX = 0f;
    private float currentLinearZ = 0f;
    private float currentAngularZ = 0f;

    private Rigidbody rb;

    // --- REFERENCIAS AUTOMÁTICAS ---
    private Floater floaterScript; 
    private RotateHelice leftHelice;
    private RotateHelice rightHelice;
    private RobotControllerButtons buttonsController; // Referència a l'altre script

    // Bandera de seguretat per evitar errors de "Thread" en cridar a funcions de Unity des de ROS2
    private bool triggerRelocate = false; 

    void Start()
    {
        // --- 1. AUTO-BÚSQUEDA DE COMPONENTES DEL ROBOT ---
        rb = GetComponent<Rigidbody>();
        if (rb == null) rb = GetComponentInChildren<Rigidbody>();

        // Cerca el Floater automàticament
        floaterScript = GetComponent<Floater>();
        if (floaterScript == null) floaterScript = GetComponentInChildren<Floater>();
        
        if (floaterScript != null) floaterScript.enabled = true;

        // Cerca l'script dels botons automàticament
        buttonsController = GetComponent<RobotControllerButtons>();
        if (buttonsController == null) buttonsController = GetComponentInChildren<RobotControllerButtons>();

        // Cerca i assigna les hèlixs
        RotateHelice[] helices = GetComponentsInChildren<RotateHelice>();
        foreach (RotateHelice h in helices)
        {
            Vector3 localPos = transform.InverseTransformPoint(h.transform.position);
            if (localPos.x < 0) leftHelice = h; 
            else rightHelice = h;               
        }

        // --- 2. CONFIGURACIÓN DEL SUSCRIPTOR DE ROS 2 ---
        ros2Unity = FindObjectOfType<ROS2UnityComponent>();
        if (ros2Unity != null && ros2Unity.Ok())
        {
            string uniqueNodeName = "robot_subscriber_node_" + System.Guid.NewGuid().ToString().Substring(0, 5);
            ros2Node = ros2Unity.CreateNode(uniqueNodeName);
            
            // 1. Ens subscrivim a cmd_vel
            twistSub = ros2Node.CreateSubscription<geometry_msgs.msg.Twist>(
                topicName, 
                msg => 
                {
                    currentLinearX = (float)msg.Linear.X;
                    currentLinearZ = (float)msg.Linear.Z;
                    currentAngularZ = (float)msg.Angular.Z;
                });

            // 2. Ens subscrivim a relocate
            relocateSub = ros2Node.CreateSubscription<std_msgs.msg.Bool>(
                relocateTopicName,
                msg =>
                {
                    // Si ens envien un "True" pel tòpic, aixequem la bandera
                    if (msg.Data)
                    {
                        triggerRelocate = true;
                    }
                });
                
            Debug.Log($"Suscrito automàticament a: {topicName} i {relocateTopicName}");
        }
        else
        {
            Debug.LogError("No se encontró el ROS2UnityComponent en la escena.");
        }
    }

    void Update()
    {
        // --- 0. EXECUCIÓ DE LA COMANDA RELOCATE ---
        // Ho fem al Update perquè funcioni dins del fil principal de Unity i no doni error.
        if (triggerRelocate)
        {
            triggerRelocate = false; // Abaixem la bandera perquè no s'executi en bucle
            
            if (buttonsController != null)
            {
                // Cridem a la funció del teu altre script
                buttonsController.Relocate();
                
                // També posem a zero les inèrcies actuals d'aquest script perquè no continuï avançant
                currentLinearX = 0f;
                currentLinearZ = 0f;
                currentAngularZ = 0f;
                
                Debug.Log("S'ha cridat a Relocate() des del tòpic ROS 2!");
            }
        }

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
        if (pos.y > -0.421f || pos.y < -4.945f)
        {
            pos.y = Mathf.Clamp(pos.y, -4.945f, -0.421f);
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