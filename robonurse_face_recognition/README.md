# robonurse_face_recognition

Features:
- Loads ArcFace-style ONNX embedding model (e.g., mobileface or insightface 112x112)
- Subscribes to `/camera/image_raw` and `/people/detections` (Detection2DArray)
- Crops detected faces and computes embeddings with onnxruntime
- Encrypted local face DB using Fernet (symmetric key)
- `register_face` script to register new identities from an image file

Maintainer: Ibrahim Aldabbagh <eng.ibrahim.aldabbagh@gmail.com>
