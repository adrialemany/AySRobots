using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lidar4D_frame : MonoBehaviour
{
    public Transform targetFrame; // Referencia al frame objetivo
    public Vector3 relativePosition; // Coordenadas relativas al frame objetivo

    // Update is called once per frame
    void Update()
    {
        if (targetFrame != null)
        {
            // Actualizar la posición del objeto para que sea relativa al frame objetivo
            transform.position = targetFrame.position + relativePosition;
        }
    }
}