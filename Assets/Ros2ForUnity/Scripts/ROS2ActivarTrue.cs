// Copyright 2019-2021 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;

namespace ROS2
{

/// <summary>
/// An example class provided for testing of basic ROS2 communication
/// </summary>
public class ROS2ActivarTrue : MonoBehaviour
{
    // Start is called before the first frame update
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<std_msgs.msg.Bool> bool_publisher;
    private int i;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    public void ActivateToTrue()
    {
        if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("ROS2UnityBoolTrueNode");
                bool_publisher = ros2Node.CreatePublisher<std_msgs.msg.Bool>("activar");
            }

            i++;
            std_msgs.msg.Bool msg = new std_msgs.msg.Bool();
            msg.Data = true;
            bool_publisher.Publish(msg);
        }
    }
}

}  // namespace ROS2
