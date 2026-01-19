from flask import Flask, request, jsonify
import torch
from io import BytesIO
from PIL import Image

app = Flask(__name__)

model_path = 'weight.pt'

# Load YOLO model
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)

@app.route('/detect', methods=['POST'])
def detect():
    try:
        # Read image from request
        image = Image.open(BytesIO(request.data))

        # Perform detection
        results = model(image)
        
        confidence_threshold = 0.5
        detections = results.pandas().xyxy[0].to_dict(orient='records')
        print(f"Detected Objects: {detections}")  # Debugging line

        # Filter detections and count cars
        car_count = 0
        filtered_detections = []

        for detection in detections:
            if detection['confidence'] >= confidence_threshold:
                if detection['name'] == 'car':
                    car_count += 1

                filtered_detections.append({
                    "xmin": detection['xmin'],
                    "ymin": detection['ymin'],
                    "xmax": detection['xmax'],
                    "ymax": detection['ymax'],
                    "name": detection['name'],
                    "confidence": detection['confidence']
                })

        print(f"Number of Cars: {car_count}")

        # Include car count in the response
        response = {
            "car_count": car_count,
            "detections": filtered_detections
        }


        # return jsonify({"detections": formatted_detections})
        return jsonify(filtered_detections)

    except Exception as e:
        return jsonify({"error": str(e)})

if __name__ == '__main__':
    app.run(debug=True)