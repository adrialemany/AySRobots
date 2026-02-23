using System;
using UnityEngine;

namespace FishAlive
{
    public static class FishConstant
    {
        public const int LAYER_INTERACT = 1 << 20;
        public const int LAYER_OBSTACLE = 1; //unity default layer as obstacles
        public static readonly Vector3 DefaultMouthPosition = new Vector3(0f, -0.01f, 0.06f); //This will be used if mouth bone is not found
        public const float FacingCosAng = 0.5f;
        public const float AvoidanceAngleStep = 20f;
        public const int AvoidanceMaxTries = 5;
        public const float AvoidanceTimerInterval = 0.2f;
    }
}