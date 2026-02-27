# Real-Time Computer Vision & Gesture Interaction System

A real-time computer vision application built with **p5.js** that processes live webcam input to demonstrate image processing pipelines, colour-space analysis, face-aware transformations, and touchless gesture interaction.

This project explores how classical computer vision techniques can enable **human–computer interaction without physical input devices**.

---

## Key Capabilities

### Image Processing Pipeline
- Snapshot capture and scaling to 160×120 processing resolution
- Grayscale conversion with brightness normalization
- RGB channel decomposition and analysis
- Per-channel thresholding with interactive sliders

### Colour Space Analysis
- RGB → HSV conversion (perceptual colour representation)
- RGB → YCbCr conversion (luminance–chrominance separation)
- Thresholding on:
  - HSV Value channel
  - YCbCr luminance (Y) channel
- Demonstrates how segmentation varies across colour models

### Face Detection & Pixel-Level Transformations
- Face detection using objectdetect (Haar-like cascade approach)
- Manual pixel operations (no built-in filters):
  - Grayscale face replacement
  - Horizontal flip via pixel remapping
  - Pixelation using 5×5 block averaging and circle rendering

### Gesture-Controlled Drawing (Extension)
- Colour segmentation to track a neon-green marker
- Centroid tracking with temporal smoothing
- Touchless drawing and erasing modes
- Demonstrates real-time vision-based interaction

---

## Technologies & Concepts

- **JavaScript (p5.js)** for real-time rendering  
- Classical **computer vision** techniques  
- Pixel-level image processing  
- Colour space transformations  
- Object detection (Haar cascade)  
- Gesture interaction via colour segmentation  

---

## How to Run

1. Open the project folder.
2. Start a local server (recommended) or open `index.html` in a browser.
3. Allow webcam access.

### Controls

| Key | Function |
|-----|---------|
| **S** | Capture snapshot |
| **1** | Grayscale face |
| **2** | Flip face |
| **3** | Pixelate face |
| **D** | Toggle gesture drawing |
| **E** | Toggle eraser |
| **C** | Clear drawing |

---

## Applications

This project demonstrates techniques applicable to:

- Real-time computer vision systems  
- Gesture-based user interfaces  
- Accessibility and touchless interaction  
- Interactive media and creative coding  

---

## Project Context

Developed as part of a computer vision exploration focusing on real-time image processing and human-computer interaction.

---

## Author

**Mansour Hammour**