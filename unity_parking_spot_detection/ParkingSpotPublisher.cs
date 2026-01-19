using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using std_msgs.msg;

namespace AWSIM
{
    /// <summary>
    /// Publish the list of empty parking spots to multiple ROS2 topics as a comma-separated string.
    /// </summary>
    public class ParkingSpotRos2Publisher : MonoBehaviour
    {
        [Tooltip("List of ROS 2 topic names to publish to.")]
        public List<string> topicNames = new List<string> { "/avp/parking_spots" };

        public QoSSettings qosSettings = new QoSSettings()
        {
            ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
            DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
            HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST,
            Depth = 10,
        };

        private List<IPublisher<std_msgs.msg.String>> _publishers = new List<IPublisher<std_msgs.msg.String>>();
        private YoloIntegration _yoloIntegration;

        private ISubscription<std_msgs.msg.String> _removalSub;
        private ISubscription<std_msgs.msg.String> _reservationSub;

        private List<int> _currentEmptySpots = new List<int>();
        private HashSet<int> _reservedSpots = new HashSet<int>();


        private void Awake()
        {
            CreatePublishers();
        }

        private void Start()
        {
            ConnectYoloIntegration();

            _removalSub = SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                "avp/reserved_parking_spots/remove",
                msg =>
                {
                    if (int.TryParse(msg.Data.Trim(), out int spotToRemove))
                    {
                        if (_currentEmptySpots.Contains(spotToRemove))
                        {
                            _currentEmptySpots.Remove(spotToRemove);
                            Debug.Log($"Removed spot {spotToRemove} from Unity list.");
                            Republish();
                        }
                    }
                });
                
            _reservationSub = SimulatorROS2Node.CreateSubscription<std_msgs.msg.String>(
                "/avp/reserved_parking_spots",
                msg =>
                {
                    string data = msg.Data;
                    int startIndex = data.IndexOf('[');
                    int endIndex = data.IndexOf(']');

                    if (startIndex != -1 && endIndex != -1 && endIndex > startIndex)
                    {
                        string listContent = data.Substring(startIndex + 1, endIndex - startIndex - 1);

                        _reservedSpots.Clear();

                        foreach (var s in listContent.Split(','))
                        {
                            if (int.TryParse(s.Trim(), out int reservedSpot))
                                _reservedSpots.Add(reservedSpot);
                        }

                        FilterReservedSpotsAndRepublish();
                    }
                });
        }

        private void CreatePublishers()
        {
            var qos = qosSettings.GetQoSProfile();

            foreach (var topic in topicNames)
            {
                var pub = SimulatorROS2Node.CreatePublisher<std_msgs.msg.String>(topic, qos);
                _publishers.Add(pub);
                Debug.Log($"Created publisher for topic: {topic}");
            }
        }

        private void ConnectYoloIntegration()
        {
            _yoloIntegration = GetComponent<YoloIntegration>();
            if (_yoloIntegration == null)
            {
                Debug.LogError("YoloIntegration component not found. Please add it to the GameObject.");
                return;
            }

            _yoloIntegration.OnParkingSpotsUpdated += Publish;
        }

        private void Publish(string emptySpots)
        {
            var newSpots = new List<int>();

            foreach (var s in emptySpots.Split(','))
            {
                if (int.TryParse(s.Trim(), out int spot) && !_reservedSpots.Contains(spot))
                {
                    newSpots.Add(spot);
                }
            }

            _currentEmptySpots = newSpots;
            Republish();
        }

        private void FilterReservedSpotsAndRepublish()
        {
            _currentEmptySpots.RemoveAll(spot => _reservedSpots.Contains(spot));
            Republish();
        }

        private void Republish()
        {
            string formatted = "[" + string.Join(", ", _currentEmptySpots) + "]";
            var msg = new std_msgs.msg.String { Data = formatted };

            foreach (var pub in _publishers)
            {
                pub.Publish(msg);
            }
        }

        private void OnDestroy()
        {
            if (_yoloIntegration != null)
                _yoloIntegration.OnParkingSpotsUpdated -= Publish;

            if (_removalSub != null)
                SimulatorROS2Node.RemoveSubscription<std_msgs.msg.String>(_removalSub);


            if (_reservationSub != null)
                SimulatorROS2Node.RemoveSubscription<std_msgs.msg.String>(_reservationSub);

            foreach (var pub in _publishers)
            {
                SimulatorROS2Node.RemovePublisher<std_msgs.msg.String>(pub);
            }

            _publishers.Clear();
            GC.Collect();
        }
    }
}