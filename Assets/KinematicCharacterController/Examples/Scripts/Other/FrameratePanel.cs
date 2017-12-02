using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace KinematicCharacterController.Examples
{
    public class FrameratePanel : MonoBehaviour
    {
        public Text MainFPS;
        public Text PhysicsFPS;
        public Text AvgFPS;
        public Text AvgFPSMin;
        public Text AvgFPSMax;

        public string[] FramerateStrings = new string[2002];

        private float _mainPollingRate = 0.1f;
        private float _avgPollingRate = 1f;

        private int _mainFramesSum = 0;
        private int _avgFramesSum = 0;
        private float _mainLastPollTime = -9999f;
        private float _avgLastPollTime = -9999f;

        private float _minDeltaTimeForAvg = Mathf.Infinity;
        private float _maxDeltaTimeForAvg = Mathf.NegativeInfinity;

        void Update()
        {
            _mainFramesSum++;
            _avgFramesSum++;

            if (Time.deltaTime < _minDeltaTimeForAvg)
            {
                _minDeltaTimeForAvg = Time.deltaTime;
            }
            if (Time.deltaTime > _maxDeltaTimeForAvg)
            {
                _maxDeltaTimeForAvg = Time.deltaTime;
            }

            float timeSinceLastMainPoll = (Time.unscaledTime - _mainLastPollTime);
            if (timeSinceLastMainPoll > _mainPollingRate)
            {
                float pollingTimefactor = _mainPollingRate / timeSinceLastMainPoll;
                MainFPS.text = GetFPSString(Mathf.RoundToInt((_mainFramesSum * pollingTimefactor) / _mainPollingRate));

                _mainLastPollTime = Time.unscaledTime;
                _mainFramesSum = 0;
            }

            float timeSinceLasAvgPoll = (Time.unscaledTime - _avgLastPollTime);
            if (timeSinceLasAvgPoll > _avgPollingRate)
            {
                float pollingTimefactor = _avgPollingRate / timeSinceLasAvgPoll;
                AvgFPS.text = GetFPSString(Mathf.RoundToInt((_avgFramesSum * pollingTimefactor) / _avgPollingRate));
                AvgFPSMin.text = GetFPSString(Mathf.RoundToInt(1f / _maxDeltaTimeForAvg));
                AvgFPSMax.text = GetFPSString(Mathf.RoundToInt(1f / _minDeltaTimeForAvg));

                _avgLastPollTime = Time.unscaledTime;
                _avgFramesSum = 0;
                _minDeltaTimeForAvg = Mathf.Infinity;
                _maxDeltaTimeForAvg = Mathf.NegativeInfinity;
            }

            PhysicsFPS.text = GetFPSString(Mathf.RoundToInt(1f / Time.fixedDeltaTime));
        }

        private string GetFPSString(int fps)
        {
            if (fps < FramerateStrings.Length - 1 && fps >= 0)
            {
                return FramerateStrings[fps];
            }
            else
            {
                return FramerateStrings[FramerateStrings.Length - 1];
            }
        }
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(FrameratePanel))]
    public class FrameratePanelEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            if (GUILayout.Button("Init strings array"))
            {
                FrameratePanel fp = target as FrameratePanel;

                for (int i = 0; i < fp.FramerateStrings.Length; i++)
                {
                    if (i >= fp.FramerateStrings.Length - 1)
                    {
                        fp.FramerateStrings[i] = i.ToString() + "+";
                    }
                    else
                    {
                        fp.FramerateStrings[i] = i.ToString();
                    }
                }
            }
        }
    }
#endif
}