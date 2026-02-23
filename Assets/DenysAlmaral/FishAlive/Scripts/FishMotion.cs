using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;
using Random = UnityEngine.Random;


namespace FishAlive
{
    /// <summary>
    /// The FishMotion class manages fish movement, animations, and environmental interactions.
    /// </summary>
    [RequireComponent(typeof(Animator))]
    public class FishMotion : MonoBehaviour
    {
        [Tooltip("The GameObject that the fish will try to swim towards or interact with.")]
        public GameObject target = null;
        [SerializeField, Tooltip("Determines the behavior of the fish when it reaches the target (e.g., Wander, Position, Bite).")] 
        private ReachMode _targetReachMode = ReachMode.Wander;
        [SerializeField, Tooltip("Reference to the SwimConfig ScriptableObject containing movement and animation parameters.")] 
        private SwimConfig Config;        
        public Action<FishMotion> OnTargetReached = null;
        public Action<FishMotion> OnBitingEnds = null;
        public Action<FishMotion> OnBiting = null;
        [Tooltip("Settings for drawing debug information in the Scene view.")]
        public DrawDebug drawDebug = new();
                
        private Animator _animator;
        [SerializeField, Tooltip("If enabled, the fish moves automatically using its internal logic. If disabled, motion must be controlled externally.")] 
        private bool _autoMotion = true;
        [SerializeField, Tooltip("Enables or disables the obstacle avoidance system.")] 
        private bool _avoidanceEnabled = false;


        public SwimState State { get { return _swimState; }}

        public bool BiteAtReach { get; set; }
        public ReachMode ReachMode => _targetReachMode;
        public float Speed => _speed;
        public bool Turning => _turning;

        private const float BiteTimeNormalized = 0.55f;
        
        private float _bodyRadius = 0.08f;
        private bool _hardLimitsEnabled = false;
        private Vector3 _hardLimitsMax = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        private Vector3 _hardLimitsMin = new Vector3(float.MinValue, float.MinValue, float.MinValue);

        private const float VerticalPrecisionMul = 0.2f;
        private float _targetPingTimer = 0;
        private bool _targetPingActive = true;
        private Vector3 _lastTargetPosition;
        private Quaternion _lastTargetRotation;
        private bool _isCloseToTarget = false;

        private int swimSpeedParam;
        private int bendParam;
        private float acceleration;
        [HideInInspector]
        public float finalAcceleration;
        private const float accelFadeInSpeed = 20.0f;
        private bool _turning = false;
        private Vector3 _turnTarget;
        private float _speed;
        private float _startTime;
        private Transform _transform;
        private Quaternion _turnTargetRotation;
        private Quaternion _turnStartRotation;
        private float _turnAngleTotal;
        private float _turningSide;
        private float _currTurningAngle;
        private float _currTurningVelocity;
        private float _turningVelocityAcceleration;
        private bool _abortingTurn = false;
        private Action _onTurnEnded = null;

        private bool _applyTargetRng = true;
        private MotionBendTracking _motionBendTracking;

        private SwimState _swimState;
        private Vector3 _mouthPositionLocal;
        private Vector3 _bitePositionLocal;
        private Dictionary<string, AnimationClip> _clips;
        private Vector3 _storedTurnDirection;

        //private Reach_OLD _reach = new Reach_OLD();
        private ReachPoint _reach = new();
        private Avoidance _avoid;
        private int _lastMarkTime;
        private Queue<(Vector3 start, Vector3 end, Color col)> _pathDebug = new();
        private int _pathDebugCapacity = 100;
        private int _frameCounter;
        private float _avoidanceTimer = 0.0f;
        private float _avoidanceTimerInterval;

        [Serializable]
        public class DrawDebug
        {
            [Tooltip("Draw visual indicators for the obstacle avoidance sensors in the Scene view.")]
            public bool avoidance = false;
            [Tooltip("Draw a trail showing the recent path taken by the fish.")]
            public bool path = false;            
        }

        private void Awake()
        {
            _transform = transform;
            _animator = GetComponent<Animator>();
            swimSpeedParam = Animator.StringToHash("swimSpeed");
            bendParam = Animator.StringToHash("turn");
            if (!Config)
            {
                Debug.LogError("A reference to Scriptable Object instance of SwimConfig is needed.", gameObject);
                enabled = false;
                return;
            }
            if (!_animator)
            {
                if (!TryGetComponent<Animator>(out _animator))
                {
                    Debug.LogError("The Animator component is missing.", gameObject);
                }
            }

            if (target)
            {
                _lastTargetPosition = target.transform.position;
            } else
            {
                _lastTargetPosition = transform.position;
            }
            GetAnimationClipsInfo();            
            _motionBendTracking = new MotionBendTracking(1024);
            CalcMouthPosition();
            _avoid = new Avoidance(_bodyRadius, _transform);
            BiteAtReach = false;

            SetAutoMotion(_autoMotion);
            SetReachMode(_targetReachMode);
        }

        void Start()
        {
            _animator.SetFloat(swimSpeedParam, 1f);
            _animator.SetFloat(bendParam, 0.5f);
            _animator.CrossFade(Animator.StringToHash("Swim"), 0, 0, Random.value);
            _swimState = SwimState.Swimming;
            acceleration = 0f;
            SetAutoMotion(_autoMotion);
            _speed = 0;
            _startTime = Time.time;                     
            _turning = false;
            if (_autoMotion) PingTarget();
            _motionBendTracking.trackingLength = _bodyRadius;
            _avoidanceTimerInterval = FishConstant.AvoidanceTimerInterval + Random.value * 0.1f;
        }

        // Update is called once per frame
        void Update()
        {
            Motion(Time.deltaTime);
            TimedCalls(Time.deltaTime);
            
            if (drawDebug.path)
            {
                MarkPath(_transform.position, _transform.forward);                
            }
        }

 


      void MarkPath(Vector3 pos, Vector3 dir)
        {
            const int MarkPathSampleFrequency = 50;
            var t = (int)(Time.time * MarkPathSampleFrequency);
            if (t != _lastMarkTime && (t % 2 == 0))
            {
                Vector3 endPos = pos + dir * 0.015f;
                var col = Color.green;
                if (_reach.Phase == 1) col = Color.blue;
                if (_reach.Phase == 2) col = Color.yellow;
                _pathDebug.Enqueue((pos, endPos, col));
                if (_pathDebug.Count > _pathDebugCapacity) _pathDebug.Dequeue();
                _lastMarkTime = t;
            }
            foreach (var rayPos in _pathDebug)
            {
                Debug.DrawLine(rayPos.start, rayPos.end, rayPos.col);
            }
        }


        void AvoidStuff()
        {
            if (_isCloseToTarget && _targetReachMode != ReachMode.Wander) return;
            _avoid.drawDebug = drawDebug.avoidance;
            if (!_avoid.Avoiding && _avoid.IsObstacleAhead(_speed))
            {
                Vector3 wayOut;
                var found = false;
                int numTries;
                (found, wayOut, numTries) = _avoid.FindAWay(_speed);
                //Debug.Log($"found way: {found}");
                if (found)
                {
                    StartTurnTowardsDirection(wayOut , true, numTries);
                    _onTurnEnded += KeepAvoiding;
                    _targetPingActive = false;
                    _avoid.Avoiding = true;
                }
            };
        }

        /// <summary>
        /// Enables or disables the fish's obstacle avoidance behavior.
        /// </summary>
        /// <param name="value">A boolean to enable (`true`) or disable (`false`) avoidance.</param>
        public void SetAvoidanceEnabled(bool value)
        {
            _avoidanceEnabled = value;
        }

        void KeepAvoiding()
        {
            _targetPingActive = true;
            _onTurnEnded -= KeepAvoiding;
            _avoid.Avoiding = false;
        }

        private void OnValidate()
        {
            //SetAutoMotion(_autoMotion);
            //SetReachMode(_targetReachMode);
        }

        /// <summary>
        /// Toggles the fish's autonomous movement. When disabled, the fish will stop moving.
        /// </summary>
        /// <param name="value">A boolean to enable (`true`) or disable (`false`) auto motion.</param>
        public void SetAutoMotion(bool value)
        {
            _autoMotion = value;
            if (_autoMotion)
            {
                finalAcceleration = Config.NormalAcceleration;
            }
            else
            {
                finalAcceleration = 0;
                acceleration = 0;
            }
        }

        /// <summary>
        /// Directly sets the current speed of the fish, overriding other motion calculations.
        /// </summary>
        /// <param name="value">The desired speed value.</param>
        public void SetSpeed(float value)
        {
            _speed = value;
        }

        /// <summary>
        /// Sets the constant forward acceleration force, influencing the fish's cruising speed.
        /// </summary>
        /// <param name="value">The desired motion force value.</param>
        public void SetMotionForce(float value)
        {            
            finalAcceleration = value;
        }

        /// <summary>
        /// Searches through the hierarchy of a transform to find a child by its name.
        /// </summary>
        /// <param name="parent">The starting transform for the search.</param>
        /// <param name="childName">The name of the child transform to find.</param>
        public Transform FindChildRecursively(Transform parent, string childName)
        {
            if (parent.name == childName)
                return parent;

            foreach (Transform child in parent)
            {
                Transform found = FindChildRecursively(child, childName);
                if (found != null)
                    return found;
            }

            return null; // If no child found
        }

        /// <summary>
        /// Updates the fish's target position and adjusts its speed and turning to move towards it.
        /// </summary>
        public void PingTarget()
        {
            if (target)
            {
                _lastTargetPosition = target.transform.position;
                _lastTargetRotation = target.transform.rotation;            
            } else 
            {
                _lastTargetRotation = _transform.rotation;
            }

            var targetPos = _lastTargetPosition;
            if (_applyTargetRng)
            {
                Vector3 rng = Random.insideUnitSphere * Config.TargetPositionPrecision;
                rng.y *= VerticalPrecisionMul;
                targetPos += rng;
            }
            if (_swimState == SwimState.Swimming) StartTurnTowardsTarget(targetPos);
            var toTarget = targetPos - _transform.position;
            var d = toTarget.magnitude;
            var facingTarget = Vector3.Dot(toTarget, _transform.forward) > FishConstant.FacingCosAng;
            _isCloseToTarget = false;

            // ------------------------------ FAR
            if (d > Config.TargetFar)
            {
                //go fast only if facing target;
                if (facingTarget) finalAcceleration = Config.FastAcceleration;

            }
            // ------------------------------ MID
            else
            {
                finalAcceleration = Config.NormalAcceleration;
                if (_targetReachMode != ReachMode.Wander) _applyTargetRng = false;
            }

            //------------------------------- CLOSE
            if (d < Config.TargetClose && (Time.time - _startTime > 0.5f))
            {
                finalAcceleration = Config.SlowAcceleration;
                _isCloseToTarget = true;
                if (_targetReachMode != ReachMode.Wander)
                {
                    //TODO: find potential issue when setting ReachMode and _isCloseToTarget is already true
                    BeginReach();
                }

            }
        }


        private void OnTriggerEnter(Collider other)
        {
             Debug.Log(other.gameObject.name);
            //var otherDir = other.transform.position - _transform.position;
            //var upAxis = Vector3.Cross(otherDir, _transform.forward);
            //if (upAxis.sqrMagnitude > 0 )
            //{
            //    var avoidRot = Quaternion.AngleAxis(45f, upAxis);
            //    var rotTarget = avoidRot * _transform.forward;
            //    ResetTurning();
            //    StartTurnTowards(rotTarget);
            //}
        }

        void CalcMouthPosition()
        {
            _mouthPositionLocal = _transform.InverseTransformPoint(FishConstant.DefaultMouthPosition);
            var t = FindChildRecursively(_transform, Config.BoneMouth);
            if (t != null)
            {
                if (_clips.TryGetValue(Config.AnimClipBite, out var biteAnim))
                {
                    biteAnim.SampleAnimation(gameObject, biteAnim.length * BiteTimeNormalized);
                    _mouthPositionLocal = _transform.InverseTransformPoint(t.position);
                    biteAnim.SampleAnimation(gameObject, 0);
                }
                else
                {
                    Debug.Log(Config.AnimClipBite + " animation missing");
                }
            }
            else Debug.Log(Config.BoneMouth + " missing");

            // Adjust for scale here, once, regardless of how _mouthPositionLocal was set
            _mouthPositionLocal = new Vector3(
                _mouthPositionLocal.x * _transform.localScale.x,
                _mouthPositionLocal.y * _transform.localScale.y,
                _mouthPositionLocal.z * _transform.localScale.z
            );

            _bodyRadius = _mouthPositionLocal.magnitude;
        }


        /// <summary>
        /// Initiates the biting animation and behavior if the fish is currently stopped.
        /// </summary>
        public void DoBite()
        {
            if (_swimState == SwimState.Stopped)
            {
                _swimState = SwimState.Biting;
                PlayBiteAnimation();
            }
        }

        void BitingMoment()
        {
            OnBiting?.Invoke(this);
        }

        void BiteEnds()
        {
            //Debug.Log("Bite ends.");
            _swimState = SwimState.Stopped;
            // _animator.CrossFade("Swim", 0.3f);
            OnBitingEnds?.Invoke(this);
        }

       private void PlayBiteAnimation()
        {
            StartCoroutine(PlayBiteAnimationCoroutine());
        }

        IEnumerator PlayBiteAnimationCoroutine()
        {
            _animator.CrossFade(Config.AnimClipBite, 0.1f); // Adjust crossfade time if needed            

            // Retrieve the duration of the animation clip
            float clipDuration = 1;
            AnimationClip clip;
            if (_clips.TryGetValue(Config.AnimClipBite, out clip)) clipDuration = clip.length;

            // Calculate the actual time to wait before triggering the end event
            float waitTime = clipDuration * 0.9f;
            var midPoint = BiteTimeNormalized;
            yield return new WaitForSeconds(waitTime * midPoint);
            BitingMoment();
            yield return new WaitForSeconds(waitTime * (1 - midPoint));
            BiteEnds();
        }


        void OnDrawGizmos()
        {     
            if (_avoid != null && drawDebug.avoidance)
            {                
                _avoid.DrawGizmo(_speed);
            }
        }

        private void SetAnimationSwimSpeed(float speed)
        {
            //Converting the real speed to a value between 0-1 to fit adecuately in the animation...
            //that's why some values here are magically hard-coded as a result of trial and error
            //TODO: use AnimationCurve later maybe
            speed *= 2f;
            speed += acceleration / 4;
            if (speed <= 0)
            {
                _animator.speed = 1.0f;
                _animator.SetFloat(swimSpeedParam, 0);
            }
            else if (speed >= 1.0f)
            {
                _animator.SetFloat(swimSpeedParam, 1);
                _animator.speed = speed;
            }
            else
            {
                _animator.speed = 1.0f;
                _animator.SetFloat(swimSpeedParam, speed);
            }

        }

        private float ApplySoftAcceleration(float dt, float accel)
        {
            float step = Config.AccelerationFadeInSpeed * dt;
            if (accel < finalAcceleration - step)
            {
                accel += step;
            }
            else if (accel > finalAcceleration + step)
            {
                accel -= step;
            }
            else
            {
                accel = finalAcceleration;
            }
            return accel;
        }


        void SetBendAnimation(float t)
        {  
           _animator.SetFloat(bendParam, t);
        }

        void Motion(float dt)
        {
            if (_swimState == SwimState.Stopped) return;

            if (_targetReachMode != ReachMode.Wander && _isCloseToTarget)
            {
                //reaching target with precision
                MotionReach(dt);
            }
            else
            {
                //directional motion
                acceleration = ApplySoftAcceleration(dt, acceleration);
                _speed += acceleration * dt;
                _speed -= Config.LiquidDrag * _speed * dt;
                Vector3 dir = _transform.forward;
                _transform.position += dt * _speed * dir;

                //turning towards target
                if (_turning) MotionTurning(dt);
            }

            Bending(dt);
            EnforceLimits();
            SetAnimationSwimSpeed(_speed);
        }




        private void MotionReach(float dt)
        {
            if (!_reach.Complete)
            {
                var dir = _transform.forward;                                

                (_speed, dir) = _reach.Update(_speed, dir, dt);
                _transform.position += (dir * _speed) * dt;

                if (_reach.Complete)
                {
                    DoTargetReach();
                }
             
            }
        }


        /// <summary>
        /// Sets how the fish behaves when it gets close to its target (e.g., wander, stop, bite).
        /// </summary>
        /// <param name="mode">The desired `ReachMode` for target interaction.</param>
        public void SetReachMode(ReachMode mode)
        {
            
                if (mode == ReachMode.Wander)
                {
                    _swimState = SwimState.Swimming;
                    _applyTargetRng = true;
                    _targetPingActive = true;
                }
                else
                {
                    //when not close to target, PingTarget() will call the BeginReach when close.
                    if (_isCloseToTarget && _swimState == SwimState.Swimming && !Application.isEditor)
                    {
                        BeginReach();                      
                    }
                    _applyTargetRng = false;
                }

                _targetReachMode = mode;
           
        }

        private void BeginReach()
        {
            _targetPingActive = false;
            ResetTurning();
            _swimState = SwimState.Reaching;
            _reach.BeginReach(_lastTargetPosition, _transform, _speed, _mouthPositionLocal);
        }

        private void Bending(float dt)
        {
            //Track the motion to have a path history
            _motionBendTracking.Add(_transform.forward, _speed * dt);
            
            //dotProd gives side and amount
            _turningSide = Vector3.Dot(_transform.right, _motionBendTracking.Tail.direction);

            //bending factor proportional to the cos(angle) between current direction and the tail of or path history       
            var factor = _turningSide * 0.8f;
            //adjust factor to 0.0 to 1.0, 0.5 is has no bend effect.
            if (factor > 0.5f) factor = 0.5f;
            if (factor < -0.5f) factor = -0.5f;
            factor = 0.5f - factor;     
            
            //apply
            SetBendAnimation(factor);
        }

        /// <summary>
        /// Begins a smooth turn towards a specified direction vector.
        /// </summary>
        /// <param name="dir">The direction vector the fish should turn towards.</param>
        /// <param name="abortExisting">If true, cancels any ongoing turn before starting the new one.</param>
        /// <param name="turnVelocityMultiplier">A multiplier to adjust the speed of the turn.</param>
        public void StartTurnTowardsDirection(Vector3 dir, bool abortExisting = false, float turnVelocityMultiplier = 1f)
        {
            const float SpeedAccelerationFactor = 800f;

            if (!_turning)
            {
                _turning = true;

                if (dir.sqrMagnitude > 0)
                {
                    dir.Normalize();

                    //rotation to face the target, and total angle
                    _turnTargetRotation = Quaternion.LookRotation(dir);
                    _turnStartRotation = _transform.rotation;
                    _turnAngleTotal = Quaternion.Angle(_turnStartRotation, _turnTargetRotation);

                    //initial values
                    _currTurningAngle = 0f;
                    _currTurningVelocity = 0f;
                    _turningVelocityAcceleration = (Config.BaseTurningAcceleration + _speed * SpeedAccelerationFactor) * turnVelocityMultiplier;
                }
            }
            else if (abortExisting)
            {
                _storedTurnDirection = dir;
                StartCoroutine(AbortAndTurnLater());

            }
        }

        /// <summary>
        /// Begins a smooth turn towards a specified world-space position.
        /// </summary>
        /// <param name="targetPosition">The world position the fish should turn to face.</param>
        /// <param name="abortExisting">If true, cancels any ongoing turn before starting the new one.</param>
        /// <param name="turnVelocityMultiplier">A multiplier to adjust the speed of the turn.</param>
        public void StartTurnTowardsTarget(Vector3 targetPosition, bool abortExisting = false, float turnVelocityMultiplier = 1f)
        {
            StartTurnTowardsDirection(targetPosition - _transform.position, abortExisting, turnVelocityMultiplier);
        }

        //
        private IEnumerator AbortAndTurnLater()
        {
            AbortTurning();
            yield return new WaitUntil(() => !_abortingTurn);
            StartTurnTowardsDirection(_storedTurnDirection);
        }

        /// <summary>
        /// Smoothly cancels the current turning maneuver, bringing the fish to a straight path.
        /// </summary>
        public void AbortTurning()
        {
            if (_turning && !_abortingTurn)
            {
                _abortingTurn = true;
                _turningVelocityAcceleration *= 3f;
                if (_turningVelocityAcceleration > 0) _turningVelocityAcceleration *= -1f;
            }
        }

        /// <summary>
        /// smooth turning of the fish towards a target over time. Accelerate until the midle, then deaccelerate.
        /// </summary>
        /// <param name="dt"></param>
        void MotionTurning(float dt)
        {
            //accelerated rotation step by step
            _currTurningAngle += _currTurningVelocity * dt;
            _currTurningVelocity += _turningVelocityAcceleration * dt;
            var turn = Quaternion.RotateTowards(_turnStartRotation, _turnTargetRotation, _currTurningAngle);
            _transform.rotation = turn;
            
            //if pass mid point, start deaccelerating
            if (_currTurningAngle > _turnAngleTotal / 2f && _turningVelocityAcceleration > 0)
            {
                _turningVelocityAcceleration = -_turningVelocityAcceleration;
            }
            //stop when 
            if (_currTurningVelocity <= 0)
            {
                if (!_abortingTurn) _onTurnEnded?.Invoke();
                ResetTurning();
            }    
        }


        private void ResetTurning()
        {
            _turning = false;
            _abortingTurn = false;
            _turningSide = 0;
        }

        /// <summary>
        /// Constrains the fish's movement within a defined bounding box.
        /// </summary>
        /// <param name="limitsMin">The minimum corner of the bounding box.</param>
        /// <param name="limitsMax">The maximum corner of the bounding box.</param>
        public void EnableHardLimits(Vector3 limitsMin, Vector3 limitsMax)
        {
            _hardLimitsEnabled = true;
            _hardLimitsMin = limitsMin + new Vector3(_bodyRadius, _bodyRadius, _bodyRadius);
            _hardLimitsMax = limitsMax - new Vector3(_bodyRadius, _bodyRadius, _bodyRadius);
        }

        /// <summary>
        /// Removes the movement constraints, allowing the fish to swim freely.
        /// </summary>
        public void DisableHardLimits()
        {
            _hardLimitsEnabled = false;
        }

        private void EnforceLimits()
        {
            if (_hardLimitsEnabled)
            {
                var p = _transform.position;
                if (p.x > _hardLimitsMax.x) p.x = _hardLimitsMax.x;
                if (p.x < _hardLimitsMin.x) p.x = _hardLimitsMin.x;
                if (p.y > _hardLimitsMax.y) p.y = _hardLimitsMax.y;
                if (p.y < _hardLimitsMin.y) p.y = _hardLimitsMin.y;
                if (p.z > _hardLimitsMax.z) p.z = _hardLimitsMax.z;
                if (p.z < _hardLimitsMin.z) p.z = _hardLimitsMin.z;
                _transform.position = p;
            }

        }

        void DoTargetReach()
        {
            _swimState = SwimState.Stopped;
            if (BiteAtReach)
            {
                DoBite();
            }
        }

        void TimedCalls(float dt)
        {
            if (_autoMotion)
            {
                if (_targetPingActive)
                {
                    _targetPingTimer -= dt;
                    if (_targetPingTimer < 0)
                    {
                        PingTarget();
                        _targetPingTimer = Config.TargetPingFrequency + Random.value * Config.TargetPingPrecision;
                    }
                }
            }

            if (_avoidanceEnabled)
            {
                _avoidanceTimer -= dt;
                if (_avoidanceTimer < 0)
                {
                    AvoidStuff();
                    _avoidanceTimer = 0.2f;
                }
            }
        }

        void GetAnimationClipsInfo()
        {
            _clips = new Dictionary<string, AnimationClip>();
            if (_animator.runtimeAnimatorController == null) return;
            AnimationClip[] clips = _animator.runtimeAnimatorController.animationClips;
            foreach (AnimationClip clip in clips)
            {
                _clips[clip.name] = clip;
            }
        }
    }

    public enum ReachMode
    {
        Wander,
        Position,
        Transform,
        PositionToBite
    }

    public enum SwimState
    {
        Swimming,
        Reaching,
        Stopped,
        Biting
    }

    public class Avoidance
    {
        private Vector3 _startPosLocal;
        private float _radius;
        private float _castDistance;
        RaycastHit _hit;
        private bool _lastCastCheck;
        public bool drawDebug = true;


        public Transform Parent { get; set; }
        public bool Avoiding { get; set; }

        public Avoidance(float bodyRadius, Transform parent)
        {
            const float CastDistanceBodyFactor = 0.6f;
            const float CastDistanceBaseOffset = 0.46f;

            _startPosLocal = new Vector3(0, 0, bodyRadius / parent.localScale.z);
            _castDistance = CastDistanceBodyFactor * bodyRadius + CastDistanceBaseOffset;
            _radius = Math.Abs(bodyRadius * 2f);
            Parent = parent;
            _lastCastCheck = false;
        }

        float RadiusCheck(float objSpeed)
        {
            return _radius * 0.5f + objSpeed * 0.1f;
        }

        public void DrawGizmo(float objSpeed)
        {
            if (!drawDebug) return;
            var p1 = Parent.TransformPoint(_startPosLocal);            
            var p2 = p1 + Parent.forward * (_castDistance * objSpeed);

            if (_lastCastCheck)
                Gizmos.color = Color.red;
            else
                Gizmos.color = Color.green;

       
                Gizmos.DrawWireSphere(p2, RadiusCheck(objSpeed) );
      
        }

        public bool IsObstacleAhead(float objSpeed)
        {
            var origin = Parent.TransformPoint(_startPosLocal);
            
            _lastCastCheck = Physics.CheckSphere(origin + _castDistance * objSpeed * Parent.forward, RadiusCheck(objSpeed), FishConstant.LAYER_OBSTACLE);
            return _lastCastCheck;
        }

  
        public (bool found, Vector3 wayOut, int numTries) FindAWay(float objSpeed)
        {
            //directions
            var dirs = new Vector3[4];
            for (int i = 0; i < dirs.Length; i++) dirs[i] = Parent.forward;         
            
            var found = false;
            var theWayOut = -Parent.forward;
            var maxTries = FishConstant.AvoidanceMaxTries;
            var tries = 0;
            var angleStep = FishConstant.AvoidanceAngleStep;
            //rotations
            var rots = new Quaternion[4];
            rots[0] = Quaternion.AngleAxis(angleStep, Parent.up); //going right
            rots[1] = Quaternion.AngleAxis(-angleStep, Parent.up); //going left
            rots[2] = Quaternion.AngleAxis(angleStep, Parent.right); //going up
            rots[3] = Quaternion.AngleAxis(-angleStep, Parent.right); //going down
            
            Vector3 origin = Parent.TransformPoint(_startPosLocal);

            do
            {
                for (int i = 0; i < dirs.Length; i++)
                {
                    dirs[i] = rots[i] * dirs[i];
                    found = !Physics.CheckSphere(origin + _castDistance * objSpeed * dirs[i], RadiusCheck(objSpeed), FishConstant.LAYER_OBSTACLE);


                    if (found)
                    {
                        if (drawDebug) Debug.DrawRay(origin, _castDistance * objSpeed * dirs[i], Color.green, 0.2f);
                        theWayOut = dirs[i];
                        found = true;
                        break;
                    }
                    else
                    {
                        if (drawDebug) Debug.DrawRay(origin, _castDistance * objSpeed * dirs[i], Color.red, 0.2f);
                    }
                }               
                
                tries++;
            } while (!found && tries < maxTries);

            return (found, theWayOut, tries);
        }

        

    }

    internal class MotionBendTracking
    {
        public struct MotionData
        {
            public Vector3 direction;
            public float length;

            public MotionData(Vector3 dir, float len)
            {
                direction = dir;
                length = len;
            }
        }

        private Queue<MotionData> _queue;
        private int _capacity;

        public int Count => _queue.Count;
        public int Capacity => _capacity;

        public float totalLength = 0;
        public float trackingLength = 10;

        public MotionData Head;
        public MotionData Tail;
        private bool _first = true;

        public MotionBendTracking(int capacity = 256)
        {
            if (capacity < 1) capacity = 10;

            _capacity = capacity;
            _queue = new Queue<MotionData>(capacity);
            Head.direction = Vector3.forward; Tail.direction = Vector3.forward;
            Head.length = 0.01f; Tail.length = 0.01f;
        }

        public void Add(Vector3 dir, float len)
        {
            if (len <= 0) return;            
            if (_queue.Count == _capacity)
            {
                RemoveOldest(); // If the queue is full, remove the oldest item                
            }

            Head = new MotionData(dir, len);
            _queue.Enqueue(Head); // Add new item to the queue
            if (_first) { 
                Tail = Head;
                _first = false;
            }
            totalLength += len;
            if (totalLength > trackingLength)
            {
                while (_queue.Count > 0 && totalLength > trackingLength)
                {
                    RemoveOldest();
                }
            }
        }

        private void RemoveOldest()
        {
            if (_queue.Count == 0)
                throw new InvalidOperationException("The buffer is empty.");

            Tail = _queue.Dequeue();
            totalLength -= Tail.length;
        }

    }



}



