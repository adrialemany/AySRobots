using UnityEngine;
using UnityEditor;


namespace DenysAlmaral
{
    //[CreateAssetMenu(fileName = "Documentation", menuName = "ReadmeDocs")]
    public class ReadmeReader : ScriptableObject
    {
        public TextAsset sourceText;
        public SceneAsset sceneDemo;
    }
}