using System;

[Serializable]
public class Detection
{
    public string name; // The label of the detected object
    public float confidence; // The confidence score
    public float xmin; // Bounding box's minimum x-coordinate
    public float ymin; // Bounding box's minimum y-coordinate
    public float xmax; // Bounding box's maximum x-coordinate
    public float ymax; // Bounding box's maximum y-coordinate
}