
using UnityEngine;

namespace FishAlive
{

    [CreateAssetMenu(fileName = "New Swim Config", menuName = "Fish Alive/Swim Config")]
    public class SwimConfig : ScriptableObject
    {
        [Min(0)]
        [Tooltip("Base frequency in seconds for how often the fish re-evaluates its target's position.")]
        public float TargetPingFrequency = 0.5f;
        [Min(0)]
        [Tooltip("Adds a random time variation to the Target Ping Frequency, making movement less predictable.")]
        public float TargetPingPrecision = 0.3f;  //seconds    
        [Min(0)]
        [Tooltip("Adds a random spatial offset to the target's position, causing the fish to swim slightly off-course.")]
        public float TargetPositionPrecision = 0.4f;
        [Min(0)]
        [Tooltip("Distance threshold. When the fish is farther than this from its target, it will use Fast Acceleration.")]
        public float TargetFar = 2f;
        [Tooltip("Distance threshold. When the fish is closer than this, it uses Slow Acceleration and may begin its final approach.")]
        public float TargetClose = 0.5f;        
        [Min(0)]
        [Tooltip("The acceleration applied when the fish is far from its target.")]
        public float FastAcceleration = 3.0f;
        [Min(0)]
        [Tooltip("The standard acceleration used when the fish is at a medium distance from its target.")]
        public float NormalAcceleration = 1.0f;
        [Range(1, 200)]
        [Tooltip("Controls how smoothly the fish changes speed. Higher values result in more abrupt acceleration changes.")]
        public float AccelerationFadeInSpeed = 20.0f;
        [Min(0)]
        [Tooltip("The acceleration applied when the fish is close to its target.")]
        public float SlowAcceleration = 0.2f;
        [Range(10, 1000)]
        [Tooltip("The base turning agility of the fish. Affects how quickly it can change direction.")]
        public float BaseTurningAcceleration = 100;
        [Min(0)]
        [Tooltip("Simulates water resistance, slowing the fish down over time. Higher values mean more drag.")]
        public float LiquidDrag = 2f;

        [Tooltip("The name of the bite animation clip in the fish's Animator Controller.")]
        public string AnimClipBite = "bite";
        [Tooltip("The name of the mouth bone transform in the fish's armature. Used to determine the bite position.")]
        public string BoneMouth = "bone_mouth";

        public SwimConfig()
        {
           
        }


    }
}