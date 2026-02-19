using UnityEngine;
using UnityEngine.UI;
using TMPro;
using ROS2;
using std_msgs.msg;

public class TerminalInput : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<String> terminalInputPublisher;
    private ISubscription<String> terminalOutputSubscriber;

    [Header("UI Configuration")]
    public TMP_InputField terminalInputField;
    public Button sendButton;

    [Header("Terminal Output")]
    public TMP_Text terminalOutputText;

    private System.Collections.Concurrent.ConcurrentQueue<string> messageQueue = new System.Collections.Concurrent.ConcurrentQueue<string>();

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
        sendButton.onClick.AddListener(OnSendButtonClicked);
    }

    void Update()
    {
        if (ros2Unity.Ok() && ros2Node == null)
        {
            ros2Node = ros2Unity.CreateNode("terminal_input_output_node");
            terminalInputPublisher = ros2Node.CreatePublisher<String>("/gui/terminal_input");
            terminalOutputSubscriber = ros2Node.CreateSubscription<String>("/gui/terminal_output", OnTerminalOutputReceived);
        }

        // Procesa mensajes desde la cola para actualizar la interfaz
        while (messageQueue.TryDequeue(out string message))
        {
            if (terminalOutputText != null)
            {
                if (string.IsNullOrEmpty(terminalOutputText.text))
                {
                    terminalOutputText.text = message; // Primer mensaje
                }
                else
                {
                    terminalOutputText.text += "\n" + message; // Agregar en nueva línea
                }

                // Ajustar el scroll para mostrar siempre el último mensaje
                var scrollRect = GetComponent<ScrollRect>();
                if (scrollRect != null)
                {
                    Canvas.ForceUpdateCanvases();
                    scrollRect.verticalNormalizedPosition = 0f;
                }
            }
        }
    }

    public void OnSendButtonClicked()
    {
        if (terminalInputPublisher != null && !string.IsNullOrWhiteSpace(terminalInputField.text))
        {
            String msg = new String
            {
                Data = terminalInputField.text
            };

            terminalInputPublisher.Publish(msg);
            Debug.Log($"Mensaje enviado: {msg.Data}");

            terminalInputField.text = string.Empty;
        }
    }

    private void OnTerminalOutputReceived(String msg)
    {
        messageQueue.Enqueue(msg.Data);
    }
}
