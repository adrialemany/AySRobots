using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement; // Required for opening scenes
using System.Linq;

namespace DenysAlmaral
{

    [CustomEditor(typeof(ReadmeReader))]
    public class ReadmeReaderEditor : Editor
    {
        private GUIStyle _titleStyle;
        private GUIStyle _descStyle;
        private GUIStyle _linkStyle;
        private bool _references = false;

        public override void OnInspectorGUI()
        {

            ReadmeReader script = (ReadmeReader)target;
            if (script.sourceText == null) return;

            // --- Parsing Logic ---
            var lines = script.sourceText.text
                .Split(new[] { "\r\n", "\r", "\n" }, System.StringSplitOptions.None)
                .ToList();

            int urlIndex = lines.FindLastIndex(l => !string.IsNullOrWhiteSpace(l));
            if (urlIndex < 1) return;

            string title = lines[0].Trim();
            string url = lines[urlIndex].Trim();

            var descLines = lines
                .GetRange(1, urlIndex - 1)
                .SkipWhile(l => string.IsNullOrWhiteSpace(l))
                .ToList();

            string description = string.Join("\n", descLines);

            SetupStyles();

            // --- UI Drawing ---
            EditorGUILayout.Space(10);
            EditorGUILayout.BeginVertical();

            EditorGUILayout.LabelField(title, _titleStyle);
            EditorGUILayout.Space(2);

            if (!string.IsNullOrEmpty(description))
            {
                EditorGUILayout.LabelField(description, _descStyle);
            }
            EditorGUILayout.Space(8);

            EditorGUILayout.BeginHorizontal();
            // Documentation Link
            if (GUILayout.Button(new GUIContent("Read Docs Online", url), _linkStyle))
            {
                Application.OpenURL(url);
            }
            EditorGUIUtility.AddCursorRect(GUILayoutUtility.GetLastRect(), MouseCursor.Link);
            // Scene Button (Only shows if a scene is assigned)
            if (script.sceneDemo != null)
            {
                GUI.backgroundColor = new Color(0.7f, 1f, 0.7f); // Light green hint
                if (GUILayout.Button(new GUIContent("Open Demo Scene", script.sceneDemo.name), GUILayout.Height(30)))
                {
                    OpenScene(script.sceneDemo);
                }
                GUI.backgroundColor = Color.white;
            }
            EditorGUILayout.EndHorizontal();


            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(20);

            _references = EditorGUILayout.BeginFoldoutHeaderGroup(_references, "References");
            if (_references)
            {
                // Draw the default Inspector (Source Text & Scene To Open fields)
                base.OnInspectorGUI();

            }
            EditorGUILayout.EndFoldoutHeaderGroup();
        }

        private void OpenScene(SceneAsset scene)
        {
            string path = AssetDatabase.GetAssetPath(scene);

            EditorApplication.delayCall += () =>
            {
                if (EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
                {
                    EditorSceneManager.OpenScene(path);
                }
            };
        }


        private void SetupStyles()
        {
            if (_titleStyle == null)
            {
                _titleStyle = new GUIStyle(EditorStyles.boldLabel) { fontSize = 22, wordWrap = true };
            }
            if (_descStyle == null)
            {
                _descStyle = new GUIStyle(EditorStyles.label) { fontSize = 14, wordWrap = true, richText = true };
            }
            if (_linkStyle == null)
            {
                _linkStyle = new GUIStyle(EditorStyles.label)
                {
                    wordWrap = false,
                    richText = true,
                    fontSize = 16,
                    fontStyle = FontStyle.Bold
                };
                Color linkColor = new Color(0.0f, 0.47f, 0.85f);
                _linkStyle.normal.textColor = linkColor;
                _linkStyle.hover.textColor = linkColor;
            }
        }
    }
}