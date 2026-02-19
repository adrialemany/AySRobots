using System.Collections.Concurrent;
using UnityEngine;
using TMPro;
using ROS2;
using std_msgs.msg;

public class IPUpdater : MonoBehaviour
{
    public TMP_Text ipTextBox; // Asocia la caja de texto TMP desde el inspector
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private bool isNodeInitialized = false;

    private ConcurrentQueue<string> ipQueue = new ConcurrentQueue<string>();

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        if (ros2Unity == null)
        {
            Debug.LogError("ROS2UnityComponent no encontrado en el GameObject.");
            return;
        }

        Debug.Log("Esperando para inicializar ROS 2...");
    }

    void Update()
    {
        // Inicializar el nodo si ROS 2 está listo
        if (ros2Unity.Ok() && !isNodeInitialized)
        {
            Debug.Log("Inicializando nodo ROS 2...");
            ros2Node = ros2Unity.CreateNode("ip_updater_node");
            ros2Node.CreateSubscription<std_msgs.msg.String>("/ip_address", OnIPReceived);
            isNodeInitialized = true;
            Debug.Log("Nodo ROS 2 inicializado y suscrito a /ip_address.");
        }

        // Procesar las IPs recibidas desde la cola
        while (ipQueue.TryDequeue(out string receivedIP))
        {
            if (IsValidIP(receivedIP))
            {
                if (ipTextBox != null)
                {
                    ipTextBox.text = $"{receivedIP}";
                }
                else
                {
                    Debug.LogError("La caja de texto TMP no está asignada.");
                }
            }
            else
            {
                Debug.LogWarning($"IP inválida recibida: {receivedIP}");
            }
        }
    }

    private void OnIPReceived(std_msgs.msg.String msg)
    {
        ipQueue.Enqueue(msg.Data); // Añadir la IP a la cola para procesarla en el hilo principal
    }

    private bool IsValidIP(string ip)
    {
        if (string.IsNullOrEmpty(ip))
            return false;

        if (System.Net.IPAddress.TryParse(ip, out System.Net.IPAddress address))
        {
            return address.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork;
        }
        return false;
    }
}
