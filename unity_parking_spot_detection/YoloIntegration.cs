// YoloIntegration.cs
using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Net.Http;
using System.Threading.Tasks;
using UnityEngine.UI;
using System.Linq;

public class YoloIntegration : MonoBehaviour
{   
    public Camera cameraToCapture; // The camera capturing the view
    public GameObject boundingBoxPrefab; // Prefab for bounding boxes
    [SerializeField] private RectTransform overheadCameraView;

    private string serverUrl = "http://127.0.0.1:5000/detect"; // YOLO server URL
    private List<GameObject> boundingBoxes = new List<GameObject>();

    [SerializeField]
    public List<ParkingSpot> parkingSpots = new List<ParkingSpot>(); // List of parking spots
    
    public List<string> emptyParkingSpots = new List<string>(); // List of empty parking spot IDs

    public event Action<string> OnParkingSpotsUpdated;

    private bool isCoroutineRunning = false;

    void Start()
    {
        Debug.Log($"Parking Spots Found: {parkingSpots.Count}");
    }

    void Update()
    {
        if (!isCoroutineRunning)
        {
            StartCoroutine(CaptureAndSendRoutine());
            isCoroutineRunning = true;
        }
    }

    void Awake()
    {
        // Find all ParkingSpot components in the scene
        ParkingSpot[] allSpots = FindObjectsOfType<ParkingSpot>();

        // Sort by number extracted from GameObject name (e.g., ParkingSpot_12 → 12)
        parkingSpots = allSpots
            .OrderBy(spot =>
            {
                string[] parts = spot.name.Split('_');
                if (parts.Length == 2 && int.TryParse(parts[1], out int number))
                    return number;
                return int.MaxValue; // if format invalid, push to end
            })
            .ToList();

        Debug.Log($"[YoloIntegration] Auto-filled and sorted {parkingSpots.Count} parking spots.");
    }

    private IEnumerator CaptureAndSendRoutine()
    {   
        yield return new WaitForSeconds(2f);

        while (true) 
        {
            CaptureAndSend(); 
            yield return new WaitForSeconds(2f); // Wait for 2 seconds before calling again
        }
    }

    Texture2D ResizeTexture(Texture2D source, int newWidth, int newHeight)
    {
        RenderTexture rt = RenderTexture.GetTemporary(newWidth, newHeight);
        Graphics.Blit(source, rt);
        RenderTexture.active = rt;

        Texture2D newTex = new Texture2D(newWidth, newHeight, TextureFormat.RGB24, false);
        newTex.ReadPixels(new Rect(0, 0, newWidth, newHeight), 0, 0);
        newTex.Apply();

        RenderTexture.ReleaseTemporary(rt);
        return newTex;
    }

    async void CaptureAndSend()
    {
        RenderTexture renderTexture = cameraToCapture.targetTexture;
        Texture2D tex = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);
        RenderTexture.active = renderTexture;
        tex.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        tex.Apply();

        Texture2D resizedTex = ResizeTexture(tex, 416, 416);
        byte[] imageBytes = tex.EncodeToPNG();
        string yoloResponse = await SendToYOLO(imageBytes);

        if (!string.IsNullOrEmpty(yoloResponse))
        {
            ParseAndDrawBoundingBoxes(yoloResponse, tex.width, tex.height);
        }

        Destroy(tex);
        RenderTexture.active = null;
    }

    async Task<string> SendToYOLO(byte[] imageBytes)
    {
        using (HttpClient client = new HttpClient())
        {
            try
            {
                ByteArrayContent content = new ByteArrayContent(imageBytes);
                HttpResponseMessage response = await client.PostAsync(serverUrl, content);
                response.EnsureSuccessStatusCode();
                return await response.Content.ReadAsStringAsync();
            }
            catch (HttpRequestException e)
            {
                Debug.LogError($"YOLO Server Error: {e.Message}");
                return null;
            }
        }
    }

    void ParseAndDrawBoundingBoxes(string json, int imageWidth, int imageHeight)
    {
        foreach (GameObject box in boundingBoxes)
        {
            Destroy(box);
        }
        boundingBoxes.Clear();

        RectTransform cameraViewRect = overheadCameraView.GetComponent<RectTransform>();
        Vector2 viewSize = cameraViewRect.rect.size;

        Detection[] detections = JsonHelper.FromJson<Detection>(json);

        // Clear the empty parking spots list before recalculating
        emptyParkingSpots.Clear();

        // Check detections for each parking spot
        foreach (ParkingSpot spot in parkingSpots)
        {
            if (spot == null) continue;

            spot.IsOccupiedByYOLO(detections.ToList(), imageWidth, imageHeight, spot.id); // sets IsOccupied and calls UpdateColor()

            // Draw the UI rectangle after it's updated
            spot.DrawInCameraView();

            if (!spot.IsOccupied)
            {
                emptyParkingSpots.Add(spot.id);
            }
        }

        foreach (Detection detection in detections)
        {
            float xMin = detection.xmin / imageWidth;
            float yMin = detection.ymin / imageHeight;
            float xMax = detection.xmax / imageWidth;
            float yMax = detection.ymax / imageHeight;

            float flippedYMin = 1 - yMax;
            float flippedYMax = 1 - yMin;

            float localXMin = xMin * viewSize.x;
            float localYMin = flippedYMin * viewSize.y;
            float localXMax = xMax * viewSize.x;
            float localYMax = flippedYMax * viewSize.y;

            // Calculate width and height
            float detectionWidth = localXMax - localXMin;
            float detectionHeight = localYMax - localYMin;
            float xOffset = -255.31916f;
            float yOffset = -133.89724f;

            localXMin += xOffset;
            localXMax += xOffset;
            localYMin += yOffset;
            localYMax += yOffset;

            GameObject box = Instantiate(boundingBoxPrefab, overheadCameraView);
            RectTransform rt = box.GetComponent<RectTransform>();
            rt.anchoredPosition = new Vector2(localXMin, localYMin);
            rt.sizeDelta = new Vector2(
                localXMax - localXMin + 15,
                localYMax - localYMin + 20
            );

            Text label = box.GetComponentInChildren<Text>();
            if (label != null)
            {
                label.text = $"{detection.name}";
                RectTransform labelRect = label.GetComponent<RectTransform>();
                if (labelRect != null)
                {
                    labelRect.anchorMin = new Vector2(0, 1);
                    labelRect.anchorMax = new Vector2(0, 1);
                    labelRect.pivot = new Vector2(0.5f, 1);
                    labelRect.anchoredPosition = new Vector2(75, 28);
                }
            }
            boundingBoxes.Add(box);
        }

        Debug.Log($"Empty Parking Spots: {string.Join(", ", emptyParkingSpots)}");
        OnParkingSpotsUpdated?.Invoke(string.Join(",", emptyParkingSpots));  // Pass a comma-separated string
    }
}