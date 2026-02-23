using System.Collections.Generic;
using UnityEngine;

namespace FishAlive
{
    /// <summary>
    /// This class implements a two-stage steering behavior designed to make an object
    /// smoothly approach a target and stop at a precise location
    /// </summary>
    [System.Serializable]
    public class ReachPoint
    {
        private int _phase;
        private Vector3 _targetPos;
        private Transform _trans;

        // Store start and end positions of the rays
        private float _midETA;
        private float _secondHalfETA;
        private float _reachStartTime;
        private float _speedChange;
        private float _angleSpeed;
        private Vector3 _toBiteTargetDir;
        private Quaternion _finalLookTarget;
        private Vector3 _offsetPoint;
        private int _executionCounter = 0;

        public bool Active => _phase >= 1 && _phase <= 2;
        public int Phase => _phase;
        public bool Complete => Phase == 3;

        //returns modified speed;
        public (float newSpeed, Vector3 newDir) Update(float speed, Vector3 dir, float dt)
        {

            switch (_phase)
            {
                case 0:
                    break;
                case 1:
                    var delta = _targetPos - _trans.position;
                    var lookTarget = Quaternion.LookRotation(delta);
                    _trans.rotation = Quaternion.RotateTowards(_trans.rotation, lookTarget, _angleSpeed * dt);
                    //_trans.rotation = lookTarget;
                    dir = _trans.forward;
                    speed += _speedChange * dt;
                    if (Time.time - _reachStartTime > _midETA)
                    {
                        BeginPhase2(speed);
                    }
                    break;
                case 2:
                    speed += _speedChange * dt;
                    dir = _toBiteTargetDir;
                    _trans.rotation = Quaternion.RotateTowards(_trans.rotation, _finalLookTarget, _angleSpeed * dt);
                    if (speed < 0)
                    {
                        speed = 0;
                        _phase = 3;
                    }
                    break;
                case 3:
                    break;
            }
            return (speed, dir);
        }



        public void BeginReach(Vector3 targetPos, Transform trans, float speed, Vector3 offsetPoint)
        {
            _targetPos = targetPos;
            _trans = trans;
            _offsetPoint = offsetPoint;
            var delta = _targetPos - _trans.position;
            var midSpeed = 0.1f;

            if (speed / 4 > midSpeed)
            {
                midSpeed = speed / 4;
            }

            _midETA = delta.magnitude / (speed + midSpeed);
            _secondHalfETA = delta.magnitude / midSpeed;

            _reachStartTime = Time.time;
            _speedChange = (midSpeed - speed) / (_midETA);
            var lookTarget = Quaternion.LookRotation(delta);
            var totalAngle = Quaternion.Angle(_trans.rotation, lookTarget);
            _angleSpeed = totalAngle / (_midETA * 0.7f);
            _phase = 1;
            _executionCounter++;
        }

        private void BeginPhase2(float speed)
        {
            _finalLookTarget = Quaternion.LookRotation(_targetPos - _trans.position);
            var mat = Matrix4x4.TRS(_targetPos, _finalLookTarget, Vector3.one);
            var toBiteTargetPos = mat.MultiplyPoint3x4(-_offsetPoint);

            var finalDelta = toBiteTargetPos - _trans.position;
            _toBiteTargetDir = (finalDelta).normalized;

            var endSpeed = 0.001f;
            _secondHalfETA = finalDelta.magnitude / ((speed + endSpeed) / 2);
            _speedChange = (endSpeed - speed) / _secondHalfETA;
            var totalAngle = Quaternion.Angle(_trans.rotation, _finalLookTarget);
            _angleSpeed = totalAngle / (_secondHalfETA * 0.5f);
            _phase = 2;
        }


    }
}