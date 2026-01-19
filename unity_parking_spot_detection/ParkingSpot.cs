using UnityEngine;
using System.Linq;
using System.Collections.Generic; 

public class ParkingSpot : MonoBehaviour
{
    public string id; // Spot ID (e.g., "Spot_1")
    public GameObject[] parkingLines; // References to the GameObjects representing the lines
    public Rect bounds; // Combined bounds of the parking space
    public bool IsOccupied = false; // Whether the spot is occupied

    public Camera overheadCamera;
    [SerializeField] private RectTransform overheadCameraView;

    private Renderer[] renderers;
    private GameObject fillPlane; // Plane to fill the spot with color
    
    private void Awake()
    {
        // Assign camera if missing
        if (overheadCamera == null)
        {
            GameObject cameraObject = GameObject.Find("OverheadCamera");
            if (cameraObject != null)
                overheadCamera = cameraObject.GetComponent<Camera>();
        }

        // Assign overheadCameraView if missing
        if (overheadCameraView == null)
        {
            GameObject viewObject = GameObject.Find("OverheadCameraView");
            if (viewObject != null)
                overheadCameraView = viewObject.GetComponent<RectTransform>();
        }

        // -----------------------------
        // AUTOMATIC ID AND LINE SETUP
        // -----------------------------
        // Extract number from GameObject name
        string name = gameObject.name.ToLower(); // e.g., "parkingspot_2"
        string[] parts = name.Split('_');

        if (parts.Length == 2 && int.TryParse(parts[1], out int spotNumber))
        {
            id = spotNumber.ToString();
            Debug.Log($"[AutoSetup] Parsed spotNumber = {spotNumber}");

            // Check if the parent is Row2
            Transform grandParent = transform.parent?.parent;
            bool isRow2 = grandParent != null && grandParent.name.ToLower().Contains("row2");
            Debug.Log($"[AutoSetup] Grandparent GameObject: {(grandParent != null ? grandParent.name : "NULL")}");
            Debug.Log($"[AutoSetup] isRow2 = {isRow2}");

            int lineIndex1 = isRow2 ? spotNumber + 1 : spotNumber;
            int lineIndex2 = isRow2 ? spotNumber + 2 : spotNumber + 1;

            string lineName1 = $"ParkingLine_{lineIndex1}";
            string lineName2 = $"ParkingLine_{lineIndex2}";

            Debug.Log($"[AutoSetup] Line Names: {lineName1}, {lineName2}");

            GameObject lineObj1 = GameObject.Find(lineName1);
            GameObject lineObj2 = GameObject.Find(lineName2);

            parkingLines = new GameObject[2];
            parkingLines[0] = lineObj1;
            parkingLines[1] = lineObj2;

            if (lineObj1 == null || lineObj2 == null)
            {
                Debug.LogWarning($"[AutoSetup] One or both parking lines not found — Tried {lineName1} and {lineName2}");
            }
            else
            {
                Debug.Log("[AutoSetup] Parking lines assigned successfully.");
            }
        }
        else
        {
            Debug.LogWarning($"[AutoSetup] Invalid GameObject name format: {gameObject.name}. Expected format: ParkingSpot_X");
        }
    }

    /// <summary>
    /// Transforms the parking spot bounds to the overhead camera's screen-space coordinates
    /// and draws them in the UI.
    /// </summary>
    public void DrawInCameraView()
    {
        if (overheadCamera == null || overheadCameraView == null)
        {
            Debug.LogError($"Overhead camera or camera view not assigned for {id}");
            return;
        }

        Rect adjustedBounds = new Rect(
            bounds.x,
            bounds.y,
            bounds.width,
            bounds.height
        );

        // Convert bounds to world space corners
        Vector3 bottomLeft = new Vector3(adjustedBounds.xMin, transform.position.y, adjustedBounds.yMin);
        Vector3 bottomRight = new Vector3(adjustedBounds.xMax, transform.position.y, adjustedBounds.yMin);
        Vector3 topLeft = new Vector3(adjustedBounds.xMin, transform.position.y, adjustedBounds.yMax);
        Vector3 topRight = new Vector3(adjustedBounds.xMax, transform.position.y, adjustedBounds.yMax);

        // Transform the world corners to screen space
        Vector3[] screenCorners = new Vector3[4];
        screenCorners[0] = overheadCamera.WorldToViewportPoint(bottomLeft);
        screenCorners[1] = overheadCamera.WorldToViewportPoint(bottomRight);
        screenCorners[2] = overheadCamera.WorldToViewportPoint(topLeft);
        screenCorners[3] = overheadCamera.WorldToViewportPoint(topRight); 

        // Convert viewport coordinates to UI local coordinates
        RectTransform cameraViewRect = overheadCameraView.GetComponent<RectTransform>();
        Vector2 viewSize = cameraViewRect.rect.size;
        Vector2[] uiCorners = new Vector2[4];

        for (int i = 0; i < screenCorners.Length; i++)
        {
            // Convert to UI coordinates by scaling the viewport coordinates
            uiCorners[i] = new Vector2(
                screenCorners[i].x * viewSize.x,
                screenCorners[i].y * viewSize.y
            );
        }
        // Draw the rectangle in the UI
        DrawRectangleInView(uiCorners, cameraViewRect);
    }

    private void DrawRectangleInView(Vector2[] corners, RectTransform parent)
    {
        if (corners.Length != 4)
        {
            Debug.LogError("Invalid number of corners for the rectangle.");
            return;
        }

        // Create a GameObject to represent the rectangle
        GameObject rectangle = new GameObject($"Rectangle_{id}", typeof(RectTransform));
        RectTransform rectTransform = rectangle.GetComponent<RectTransform>();
        rectTransform.SetParent(parent, false);

        // Calculate the center and size of the rectangle
        Vector2 center = (corners[0] + corners[2]) / 2;
        Vector2 size = new Vector2(
            Vector2.Distance(corners[0], corners[1]), // Width
            Vector2.Distance(corners[0], corners[2])  // Height
        );

        size.x -= 5;
        size.y -= 5;

        // Define offsets for the bounds
        float offsetX = -271.29f; // Horizontal adjustment
        float offsetY = -187.0f; // Vertical adjustment
        
        // Apply offsets to the center position
        center += new Vector2(offsetX, offsetY);

        rectTransform.anchoredPosition = center;
        // rectTransform.sizeDelta = size;

        rectTransform.sizeDelta = new Vector2(
            size.x, // Reduce width by 5
            size.y  // Reduce height by 5
        );
        
        // ***** UNCOMMENT TO SEE PARKING SPOT BOUNDS *****

        // Add a background image to visualize the rectangle
        // UnityEngine.UI.Image image = rectangle.AddComponent<UnityEngine.UI.Image>();
        // image.color = IsOccupied ? new Color(1, 0, 0, 0.5f) : new Color(0, 1, 0, 0.5f); // Red for occupied, green for empty
	
	    // -------------------------------
        // ADDING TEXT LABEL FOR SPOT ID
        // -------------------------------
        GameObject labelGO = new GameObject($"Label_{id}", typeof(RectTransform));
        RectTransform labelRect = labelGO.GetComponent<RectTransform>();
        labelRect.SetParent(parent, false);
        labelRect.anchoredPosition = center;
        labelRect.sizeDelta = new Vector2(50, 20); // Size of the label

        var text = labelGO.AddComponent<UnityEngine.UI.Text>();
        text.text = id;
        text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        text.fontSize = 24;
        text.color = Color.white;
        text.alignment = TextAnchor.MiddleCenter;
        text.horizontalOverflow = HorizontalWrapMode.Overflow;
        text.verticalOverflow = VerticalWrapMode.Overflow;
        
    }

    public void IsOccupiedByYOLO(List<Detection> detections, int imageWidth, int imageHeight, string spotID)
    {
        if (overheadCameraView == null)
        {
            Debug.LogError("Overhead camera view not assigned.");
            return;
        }

        // Transform the parking spot bounds to UI-space using DrawRectangleInView
        RectTransform cameraViewRect = overheadCameraView.GetComponent<RectTransform>();
        Vector3[] screenCorners = new Vector3[4];
        
        Vector3 bottomLeft = new Vector3(bounds.xMin, transform.position.y, bounds.yMin);
        Vector3 bottomRight = new Vector3(bounds.xMax, transform.position.y, bounds.yMin);
        Vector3 topLeft = new Vector3(bounds.xMin, transform.position.y, bounds.yMax);
        Vector3 topRight = new Vector3(bounds.xMax, transform.position.y, bounds.yMax);

        // Convert to viewport space
        screenCorners[0] = overheadCamera.WorldToViewportPoint(bottomLeft);
        screenCorners[1] = overheadCamera.WorldToViewportPoint(bottomRight);
        screenCorners[2] = overheadCamera.WorldToViewportPoint(topLeft);
        screenCorners[3] = overheadCamera.WorldToViewportPoint(topRight);

        // Convert viewport space to UI local space
        Vector2 viewSize = cameraViewRect.rect.size;
        Vector2[] uiCorners = new Vector2[4];

        for (int i = 0; i < screenCorners.Length; i++)
        {
            uiCorners[i] = new Vector2(
                screenCorners[i].x * viewSize.x,
                screenCorners[i].y * viewSize.y
            );
        }

        // Calculate the final UI rectangle
        Vector2 center = (uiCorners[0] + uiCorners[2]) / 2;

        Vector2 size = new Vector2(
            Vector2.Distance(uiCorners[0], uiCorners[1]), // Width
            Vector2.Distance(uiCorners[0], uiCorners[2])  // Height
        );

        // Offsets applied (same as in DrawRectangleInView)
        float offsetX = -271.29f;
        float offsetY = -187.0f;
        center += new Vector2(offsetX, offsetY);

        size -= new Vector2(5, 5);  // Reduce width and height by 5

        Rect uiBounds = new Rect(
            center.x,
            center.y,
            size.x,
            size.y
        );

        // Debug.Log($"[ParkingSpot] spotID = {spotID},  UI Bounds: {uiBounds}");

        // Check for overlap with YOLO detections
        bool isOccupied = false;

        foreach (var detection in detections)
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

            // Create detection rect in UI space
            Rect detectionRect = new Rect(
                localXMin,
                localYMin,
                (detection.xmax - detection.xmin) / imageWidth * viewSize.x,
                (detection.ymax - detection.ymin) / imageHeight * viewSize.y
            );

            // Check for overlap
            if (uiBounds.Overlaps(detectionRect))
            {
                isOccupied = true;
                break;
            }
        }

        IsOccupied = isOccupied;
        UpdateColor();
    }

    void Start()
    {
        renderers = new Renderer[parkingLines.Length];
        for (int i = 0; i < parkingLines.Length; i++)
        {
            if (parkingLines[i] == null)
            {
                Debug.LogWarning($"[Init] parkingLines[{i}] is null in {gameObject.name}");
                continue;
            }

            Renderer r = parkingLines[i].GetComponent<Renderer>();
            if (r == null)
            {
                Debug.LogWarning($"[Init] parkingLines[{i}] has no Renderer in {gameObject.name}");
            }

            renderers[i] = r;
        }

        bounds = CalculateBounds();

        // ***** UNCOMMENT TO SEE PARKING SPOT PLANES FOR DEBUGGING *****
        // CreateFillPlane();
    }

    void Update()
    {
        // Debug visualization: change color of lines based on occupancy
        Color color = IsOccupied ? Color.red : Color.green;
        foreach (Renderer renderer in renderers)
        {
            renderer.material.color = color;
        }
    }

    private Rect CalculateBounds()
    {
        if (parkingLines.Length < 2)
        {
            Debug.LogError($"Not enough parking lines assigned to {id}");
            return new Rect();
        }

        // To store the final combined bounds
        Bounds combinedBounds = new Bounds(parkingLines[0].transform.position, Vector3.zero);

        // Iterate through all consecutive pairs of parking lines
        for (int i = 0; i < parkingLines.Length - 1; i++)
        {
            Vector3 line1Pos = parkingLines[i].transform.position;
            Vector3 line2Pos = parkingLines[i + 1].transform.position;

            // Assuming these values are correct, but we'll manually adjust the Y scale for this calculation
            Vector3 line1Scale = new Vector3(-2.42f, 1f, 5.28f);  // Correct X and Z scale, Y scale manually adjusted
            Vector3 line2Scale = new Vector3(-2.42f, 1f, 5.28f);  // Same scale for both lines

            // Calculate the distance between the two parking lines (depth of the spot)
            float distance = Vector3.Distance(line1Pos, line2Pos);

            // Ensure that the Y-scale remains correctly adjusted
            float width = Mathf.Abs(line1Scale.x);  // Width based on the x-scale
            float depth = Mathf.Abs(line1Scale.z);  // Depth based on the z-scale
            float height = Mathf.Abs(line1Scale.y); // Manually adjust if needed, but this is typically irrelevant for bounds

            // Optionally, apply offsets to create space between adjacent spots
            float widthOffset = 0f; // Add padding width-wise
            float depthOffset = 0f; // Add padding depth-wise

            // Find the center of the parking spot (midpoint between the two parking lines)
            Vector3 center = (line1Pos + line2Pos) / 2;

            // Update the combined bounds by encapsulating the bounds of this parking spot
            combinedBounds.Encapsulate(new Bounds(center, new Vector3(width + widthOffset, height, depth + depthOffset)));        
        }

        // Convert the final combined bounds to a Rect
        Rect finalBounds = new Rect(
            combinedBounds.min.x,
            combinedBounds.min.z,  // Use Z-axis for depth in 3D
            combinedBounds.size.x,
            combinedBounds.size.z
        );

        return finalBounds;
    }

    public void UpdateColor()
    {
        if (renderers == null || renderers.Length == 0)
        {
            // Debug.LogWarning($"[UpdateColor] Renderer array is not initialized for {id}. Skipping color update.");
            return;
        }

        Color color = IsOccupied ? Color.red : Color.green;

        for (int i = 0; i < renderers.Length; i++)
        {
            if (renderers[i] == null)
            {
                Debug.LogWarning($"[UpdateColor] Renderer[{i}] is null for spot {id}. Skipping.");
                continue;
            }

            if (renderers[i].material == null)
            {
                Debug.LogWarning($"[UpdateColor] Material is null on Renderer[{i}] for spot {id}. Skipping.");
                continue;
            }

            renderers[i].material.color = color;
        }

        if (fillPlane != null)
        {
            Renderer fillR = fillPlane.GetComponent<Renderer>();
            if (fillR != null && fillR.material != null)
            {
                fillR.material.color = color;
            }
        }
    }

    private void CreateFillPlane()
    {
        if (bounds.width <= 0 || bounds.height <= 0)
        {
            Debug.LogError($"Invalid bounds for {id}: {bounds}");
            return;
        }

        // Create the plane
        fillPlane = GameObject.CreatePrimitive(PrimitiveType.Quad);
        fillPlane.name = $"FillPlane_{id}";
        fillPlane.transform.SetParent(transform, false);

        // Position and scale
        Vector3 center = new Vector3(bounds.x + bounds.width / 2, 0.01f, bounds.y + bounds.height / 2);
        fillPlane.transform.position = center;

        fillPlane.transform.localScale = new Vector3(bounds.width, bounds.height, 1);

        // Determine the average rotation of parking lines
        Quaternion averageRotation = CalculateAverageRotation();
        fillPlane.transform.rotation = averageRotation * Quaternion.Euler(90, 0, 0);
        
        // Apply a material
        Material fillMaterial = new Material(Shader.Find("Unlit/Color"));
        fillMaterial.color = Color.clear; // Initially transparent
        fillPlane.GetComponent<Renderer>().material = fillMaterial;

        // Disable the collider
        Destroy(fillPlane.GetComponent<Collider>());
    }

    private Quaternion CalculateAverageRotation()
    {
        if (parkingLines.Length == 0)
        {
            Debug.LogError($"No parking lines assigned to {id}");
            return Quaternion.identity;
        }

        Vector3 averageForward = Vector3.zero;
        Vector3 averageUp = Vector3.zero;

        foreach (GameObject line in parkingLines)
        {
            Transform lineTransform = line.transform;
            averageForward += lineTransform.forward;
            averageUp += lineTransform.up;
        }

        averageForward.Normalize();
        averageUp.Normalize();

        // Construct the average rotation
        Quaternion averageRotation = Quaternion.LookRotation(averageForward, averageUp);
        return averageRotation;
    }

    public void SetFillColor(Color color)
    {
        if (fillPlane != null)
        {
            fillPlane.GetComponent<Renderer>().material.color = color;
            Debug.Log($"Set color for {id} to {color}");
        }
        else
        {
            Debug.LogError($"Fill plane not created for {id}");
        }
    }

    public void ClearFillColor()
    {
        if (fillPlane != null)
        {
            Debug.Log($"ClearFillColor Called");
            fillPlane.GetComponent<Renderer>().material.color = Color.clear; // Reset to transparent
        }
    }

    public void TestFillColors()
    {
        if (fillPlane != null)
        {   
            Debug.Log($"TestFillColors Called");
            SetFillColor(Color.blue); // Change this to any color for testing

        }
        else
        {
            Debug.LogError($"Fill plane is not created for {id}");
        }
    }
}