#!/usr/bin/env python3
"""register_face.py

Simple CLI to register a face into the encrypted DB.
Usage:
  python register_face.py --image /path/to/image.jpg --id "patient_01"

The script will:
  - load the image
  - optionally run a simple face crop (center) or use given bbox (not implemented)
  - compute embedding using the same ONNX model
  - append to the encrypted DB (average if id exists)
"""

import argparse
import cv2
import os
import json
import numpy as np
from cryptography.fernet import Fernet
import onnxruntime as ort

def l2_normalize(x, axis=1, epsilon=1e-10):
    norm = np.linalg.norm(x, ord=2, axis=axis, keepdims=True)
    return x / (norm + epsilon)

def compute_embedding_for_image(img_path, session, crop_size):
    img = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(img_path)
    h,w = img.shape[:2]
    # naive center crop to square then resize
    minside = min(h,w)
    cy = h//2; cx = w//2
    x1 = cx - minside//2; y1 = cy - minside//2
    crop = img[y1:y1+minside, x1:x1+minside]
    face = cv2.resize(crop, (crop_size, crop_size))
    face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB).astype('float32')
    face = (face - 127.5) / 128.0
    inp = np.transpose(face, (2,0,1))[None,:,:,:].astype('float32')
    emb = session.run(None, {session.get_inputs()[0].name: inp})[0]
    emb = np.squeeze(emb).astype('float32')
    emb = emb / np.linalg.norm(emb)
    return emb

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', required=True)
    parser.add_argument('--id', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--db', required=True)
    parser.add_argument('--key', required=True)
    parser.add_argument('--crop_size', type=int, default=112)
    args = parser.parse_args()

    if not os.path.exists(args.model):
        raise FileNotFoundError(args.model)
    sess = ort.InferenceSession(args.model, providers=['CPUExecutionProvider'])

    emb = compute_embedding_for_image(args.image, sess, args.crop_size)

    # load key
    with open(args.key, 'rb') as f:
        key = f.read()
    fernet = Fernet(key)

    # load or init db
    db = {}
    if os.path.exists(args.db):
        with open(args.db, 'rb') as f:
            enc = f.read()
        dec = fernet.decrypt(enc)
        db = json.loads(dec.decode('utf-8'))
    else:
        db = {}

    if args.id in db:
        # average embeddings
        prev = np.array(db[args.id], dtype='float32')
        new = (prev + emb) / 2.0
        new = (new / (np.linalg.norm(new) + 1e-10)).tolist()
        db[args.id] = new
    else:
        db[args.id] = emb.tolist()

    # save
    data = json.dumps(db).encode('utf-8')
    enc = fernet.encrypt(data)
    with open(args.db, 'wb') as f:
        f.write(enc)
    print(f"Registered id {args.id} in DB ({len(db)} entries)")

if __name__ == '__main__':
    main()
