from ultralytics import YOLO

if __name__ == "__main__":
    # Load the YOLOv11 model (Ensure this file exists!)
    model = YOLO("yolo11n.pt")

    # Train the model on your dataset
    results = model.train(data="C:/Users/Siddh/Desktop/testmodel/data.yaml", epochs=10, imgsz=128,batch=32)

    # Run inference
    results = model("C:/Users/Siddh/Desktop/testmodel/test.jpg")

    # Show detection results
    results.show()
