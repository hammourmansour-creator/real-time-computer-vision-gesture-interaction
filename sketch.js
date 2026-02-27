/**
 * ------------------------------------------------------------
 * Commentary – Image Processing & Face Interaction App
 * ------------------------------------------------------------
 *
 * This application captures live webcam input and processes it
 * through a structured grid of image processing tasks. Pressing
 * the S key captures a 160×120 snapshot that becomes the source
 * for all subsequent operations. The grid layout ensures that
 * each task is displayed in a consistent and readable position.
 *
 * The first stage converts the snapshot to grayscale while
 * reducing brightness by 20% within a single pixel loop. Care
 * was taken to clamp pixel values to avoid underflow below zero.
 * The original image is then split into its red, green, and blue
 * channels, each displayed as intensity images. Thresholding is
 * applied to each channel using sliders ranging from 0–255,
 * allowing real-time interaction and demonstrating how binary
 * segmentation changes with different thresholds.
 *
 * Two colour space conversions were implemented: HSV and YCbCr.
 * These were chosen because they represent colour in perceptual
 * and luminance-based models. Thresholding is applied to the V
 * channel in HSV and the Y channel in YCbCr, highlighting how
 * brightness-based segmentation behaves differently across
 * colour spaces.
 *
 * Face detection is implemented using the Week 19 objectdetect
 * approach. After a snapshot is taken, the strongest detected
 * face is highlighted with a bounding box. Using keys 1–3, the
 * detected face can be replaced with three filters: grayscale,
 * horizontal flip, and pixelation. These filters were implemented
 * manually using nested loops and pixel arrays rather than built-in
 * p5.js functions, ensuring compliance with the coursework
 * requirements. Pixelation divides the face into 5×5 blocks and
 * renders filled circles using the average intensity of each block.
 *
 * One challenge encountered was managing pixel operations efficiently
 * while maintaining real-time performance. Creating images inside the
 * draw loop caused lag and memory issues; this was resolved by using
 * reusable buffers. Another difficulty was ensuring that face filters
 * aligned correctly within scaled grid cells, which required careful
 * coordinate mapping.
 *
 * The project remained on schedule and all required tasks were
 * completed. If further time were available, improvements would
 * include optimizing detection for multiple faces and adding
 * performance smoothing for rapid motion.
 *
 * Extension – Gesture Drawing Tool:
 * A separate grid cell introduces an interactive drawing feature
 * controlled by hand colour tracking. When activated, the user can
 * draw on the canvas using hand movement, with a gesture-based eraser
 * mode. This extension was chosen because it transforms the webcam
 * from a passive input device into an interactive tool, combining
 * image processing, centroid tracking, and real-time rendering.
 * Unlike simple filter additions, it demonstrates a practical
 * human-computer interaction concept and expands the application
 * beyond static processing into live visual interaction.
 *
 * ------------------------------------------------------------
 */

const CANVAS_W = 640;
const CANVAS_H = 480;

const SNAP_W = 160;
const SNAP_H = 120;

let app;

// ---------- App: main controller (state + processing) ----------
class App {
  constructor() {
    // Layout and input
    this.grid = new GridLayout(3, 5, CANVAS_W, CANVAS_H);
    this.webcam = new WebcamSource();

    // Step 1–3: snapshot buffer (160×120)
    this.snapshotImg = createImage(SNAP_W, SNAP_H);
    this.hasSnapshot = false;

    // Step 4: grayscale + brightness -20%
    this.grayMinus20Img = createImage(SNAP_W, SNAP_H);

    // Step 6: RGB channel split
    this.rImg = createImage(SNAP_W, SNAP_H);
    this.gImg = createImage(SNAP_W, SNAP_H);
    this.bImg = createImage(SNAP_W, SNAP_H);
    this.rgbReady = false;

    // Step 7–8: threshold outputs + slider state
    this.rThreshImg = createImage(SNAP_W, SNAP_H);
    this.gThreshImg = createImage(SNAP_W, SNAP_H);
    this.bThreshImg = createImage(SNAP_W, SNAP_H);
    this.rThresh = 128;
    this.gThresh = 128;
    this.bThresh = 128;

    // Step 9: colour space conversion outputs
    this.hsvImg = createImage(SNAP_W, SNAP_H);
    this.ycbcrImg = createImage(SNAP_W, SNAP_H);

    // Step 10: colour-space threshold outputs + slider state
    this.hsvThreshImg = createImage(SNAP_W, SNAP_H);
    this.ycbcrThreshImg = createImage(SNAP_W, SNAP_H);
    this.hsvVThresh = 128;
    this.ycbcrYThresh = 128;

    // Sliders + labels
    this.rSlider = null;
    this.gSlider = null;
    this.bSlider = null;
    this.hsvVSlider = null;
    this.ycbcrYSlider = null;

    this.rLabel = null;
    this.gLabel = null;
    this.bLabel = null;
    this.hsvVLabel = null;
    this.ycbcrYLabel = null;

    // Step 11: face detection (objectdetect)
    this.detector = null;
    this.faceRects = []; // objectdetect detections: [x, y, w, h, confidence]
    this.minFaceConfidence = 4;

    // Step 12: face replacement mode
    // 0 = show Step 11 box only, 1 = grayscale, 2 = flip, 3 = pixelate
    this.faceMode = 0;

    // Work buffers (avoid allocations inside draw)
    this.snapshotG = createGraphics(SNAP_W, SNAP_H);
    this.snapshotG.pixelDensity(1);

        // ---------------- Extension: Hand-draw (independent cell) ----------------
    this.extEnabled = false;     // toggle with D
    this.extEraser = false;      // toggle with E
    this.extStrokeW = 3;         // pen thickness (in 160x120 space)

    // Live 160x120 source for extension 
    this.extSrcImg = createImage(SNAP_W, SNAP_H);

    // Mask + drawing layer (same resolution as processing)
    this.extMaskImg = createImage(SNAP_W, SNAP_H);
    this.extLayer = createGraphics(SNAP_W, SNAP_H);
    this.extLayer.pixelDensity(1);
    this.extLayer.clear();

    // Tracking / smoothing
    this.extPrevPt = null;
    this.extSmooth = { x: 0, y: 0, has: false };
    this.extMinPixels = 120;     

    // Neon-green thresholds in HSV (h in 0..179, s/v in 0..255)
    this.extHueMin = 50;
    this.extHueMax = 85;
    this.extSatMin = 90;
    this.extValMin = 60;

    this.faceWorkImg = createImage(SNAP_W, SNAP_H);
  }

  // ---------- App init ----------
  init() {
    this.webcam.init();
    this.initThresholdSliders();
    this.initColourSpaceThresholdSliders();
    this.initFaceDetector();
  }

  // ---------- Utility ----------
  /**
   * Clamp a numeric value into byte range [0,255].
   * Prevents intensity underflow/overflow when applying brightness operations.
   * @param {number} v
   * @returns {number}
   */
  clampByte(v) {
    return Math.max(0, Math.min(255, v));
  }

  // ---------- Step 1–3: Snapshot capture (160×120) ----------
  /**
   * Step 1–2:
   * Capture a snapshot from the webcam and scale to 160×120 pixels.
   * This snapshot becomes the source image for Steps 4–12.
   */
  takeSnapshot() {
    if (!this.webcam.isReady()) return;

    // Step 2: scale webcam frame into the 160×120 snapshot buffer
    this.snapshotImg.copy(
      this.webcam.capture,
      0,
      0,
      this.webcam.capture.width,
      this.webcam.capture.height,
      0,
      0,
      SNAP_W,
      SNAP_H
    );

    this.hasSnapshot = true;

    // Build dependent outputs immediately after snapshot capture
    this.buildGrayMinus20(this.snapshotImg, this.grayMinus20Img);
    this.updateRGBChannels();
    this.detectFaces();

    // Default to Step 11 view (red box only) until 1/2/3 is pressed
    this.faceMode = 0;
  }

  // ---------- Step 4–5: Grayscale + brightness -20% (single loop) ----------
  /**
   * Step 4–5:
   * Convert snapshot to grayscale and reduce brightness by 20%
   * in the same nested loop. The clamp prevents values falling below 0.
   *
   * @param {p5.Image} src 160×120 snapshot
   * @param {p5.Image} dst 160×120 output buffer
   */
  buildGrayMinus20(src, dst) {
    src.loadPixels();
    dst.loadPixels();

    const w = src.width;
    const h = src.height;

    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const idx = (y * w + x) * 4;

        const rr = src.pixels[idx];
        const gg = src.pixels[idx + 1];
        const bb = src.pixels[idx + 2];

        // Module-style grayscale using average intensity
        let gray = (rr + gg + bb) / 3;

        // Brightness reduction by 20% with clamp to [0..255]
        gray = this.clampByte(gray * 0.8);

        dst.pixels[idx] = gray;
        dst.pixels[idx + 1] = gray;
        dst.pixels[idx + 2] = gray;
        dst.pixels[idx + 3] = 255;
      }
    }

    dst.updatePixels();
  }

  // ---------- Step 6: RGB channel split ----------
  /**
   * Step 6:
   * Split the snapshot into separate R/G/B intensity images.
   * Each channel image is displayed as grayscale using the channel value.
   */
  updateRGBChannels() {
    if (!this.hasSnapshot) return;

    const src = this.snapshotImg;
    src.loadPixels();

    this.rImg.loadPixels();
    this.gImg.loadPixels();
    this.bImg.loadPixels();

    const w = src.width;
    const h = src.height;

    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = (x + y * w) * 4;

        const rr = src.pixels[i];
        const gg = src.pixels[i + 1];
        const bb = src.pixels[i + 2];

        // R channel intensity
        this.rImg.pixels[i] = rr;
        this.rImg.pixels[i + 1] = rr;
        this.rImg.pixels[i + 2] = rr;
        this.rImg.pixels[i + 3] = 255;

        // G channel intensity
        this.gImg.pixels[i] = gg;
        this.gImg.pixels[i + 1] = gg;
        this.gImg.pixels[i + 2] = gg;
        this.gImg.pixels[i + 3] = 255;

        // B channel intensity
        this.bImg.pixels[i] = bb;
        this.bImg.pixels[i + 1] = bb;
        this.bImg.pixels[i + 2] = bb;
        this.bImg.pixels[i + 3] = 255;
      }
    }

    this.rImg.updatePixels();
    this.gImg.updatePixels();
    this.bImg.updatePixels();

    this.rgbReady = true;
  }

  // ---------- Step 7–8: Thresholding (per channel) ----------
  /**
   * Step 7–8:
   * Threshold a single-channel intensity image into a black/white result.
   * The threshold is controlled by a slider in range 0..255.
   *
   * @param {p5.Image} src grayscale intensity image (R/G/B channel image)
   * @param {p5.Image} dst output threshold image
   * @param {number} t threshold value in [0..255]
   */
  buildThreshold(src, dst, t) {
    src.loadPixels();
    dst.loadPixels();

    const w = src.width;
    const h = src.height;

    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = (x + y * w) * 4;

        // Source is grayscale; read one channel as intensity
        const v = src.pixels[i];
        const out = v > t ? 255 : 0;

        dst.pixels[i] = out;
        dst.pixels[i + 1] = out;
        dst.pixels[i + 2] = out;
        dst.pixels[i + 3] = 255;
      }
    }

    dst.updatePixels();
  }

  // ---------- Step 9: Colour space conversion (HSV and YCbCr) ----------
  /**
   * Convert RGB [0..255] to HSV.
   * Output ranges:
   * - Hue in [0..179] (OpenCV-style range suitable for thresholding tasks)
   * - Saturation, Value in [0..255]
   *
   * @param {number} r
   * @param {number} g
   * @param {number} b
   * @returns {{h179:number, s255:number, v255:number}}
   */
  rgbToHsv(r, g, b) {
    const rn = r / 255;
    const gn = g / 255;
    const bn = b / 255;

    const cMax = Math.max(rn, gn, bn);
    const cMin = Math.min(rn, gn, bn);
    const delta = cMax - cMin;

    let hDeg = 0;
    if (delta !== 0) {
      if (cMax === rn) hDeg = 60 * (((gn - bn) / delta) % 6);
      else if (cMax === gn) hDeg = 60 * (((bn - rn) / delta) + 2);
      else hDeg = 60 * (((rn - gn) / delta) + 4);
    }
    if (hDeg < 0) hDeg += 360;

    const s = cMax === 0 ? 0 : delta / cMax;
    const v = cMax;

    const h179 = Math.round((hDeg / 360) * 179);
    const s255 = Math.round(s * 255);
    const v255 = Math.round(v * 255);

    return { h179, s255, v255 };
  }

  /**
   * Convert RGB [0..255] to YCbCr (BT.601 full-range style).
   * Output channels are clamped to [0..255].
   *
   * @param {number} r
   * @param {number} g
   * @param {number} b
   * @returns {{y:number, cb:number, cr:number}}
   */
  rgbToYCbCr(r, g, b) {
    let y = 0.299 * r + 0.587 * g + 0.114 * b;
    let cb = 128 - 0.168736 * r - 0.331264 * g + 0.5 * b;
    let cr = 128 + 0.5 * r - 0.418688 * g - 0.081312 * b;

    y = this.clampByte(y);
    cb = this.clampByte(cb);
    cr = this.clampByte(cr);

    return { y, cb, cr };
  }

  /**
   * Step 9:
   * Build HSV image for display by packing channels into RGB:
   * - R = Hue mapped from [0..179] to [0..255] for visualisation
   * - G = Saturation [0..255]
   * - B = Value [0..255]
   *
   * @param {p5.Image} src snapshot
   * @param {p5.Image} dst hsv display image
   */
  buildHSV(src, dst) {
    src.loadPixels();
    dst.loadPixels();

    const w = src.width;
    const h = src.height;

    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = (x + y * w) * 4;

        const rr = src.pixels[i];
        const gg = src.pixels[i + 1];
        const bb = src.pixels[i + 2];

        const { h179, s255, v255 } = this.rgbToHsv(rr, gg, bb);

        // Display hue in 0..255 range for visibility
        const hDisp = Math.round((h179 / 179) * 255);

        dst.pixels[i] = hDisp;
        dst.pixels[i + 1] = s255;
        dst.pixels[i + 2] = v255;
        dst.pixels[i + 3] = 255;
      }
    }

    dst.updatePixels();
  }

  /**
   * Step 9:
   * Build YCbCr image for display by packing channels into RGB:
   * - R = Y
   * - G = Cb
   * - B = Cr
   *
   * @param {p5.Image} src snapshot
   * @param {p5.Image} dst ycbcr display image
   */
  buildYCbCr(src, dst) {
    src.loadPixels();
    dst.loadPixels();

    const w = src.width;
    const h = src.height;

    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = (x + y * w) * 4;

        const rr = src.pixels[i];
        const gg = src.pixels[i + 1];
        const bb = src.pixels[i + 2];

        const { y: yy, cb, cr } = this.rgbToYCbCr(rr, gg, bb);

        dst.pixels[i] = yy;
        dst.pixels[i + 1] = cb;
        dst.pixels[i + 2] = cr;
        dst.pixels[i + 3] = 255;
      }
    }

    dst.updatePixels();
  }

  // ---------- Step 10: Thresholding from colour spaces (single channel) ----------
  /**
   * Step 10:
   * Threshold the HSV Value channel (V in [0..255]) from the original snapshot.
   *
   * @param {p5.Image} src snapshot
   * @param {p5.Image} dst threshold image
   * @param {number} t threshold in [0..255]
   */
  buildHSV_V_Threshold(src, dst, t) {
    src.loadPixels();
    dst.loadPixels();

    const w = src.width;
    const h = src.height;

    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = (x + y * w) * 4;

        const rr = src.pixels[i];
        const gg = src.pixels[i + 1];
        const bb = src.pixels[i + 2];

        const { v255 } = this.rgbToHsv(rr, gg, bb);
        const out = v255 > t ? 255 : 0;

        dst.pixels[i] = out;
        dst.pixels[i + 1] = out;
        dst.pixels[i + 2] = out;
        dst.pixels[i + 3] = 255;
      }
    }

    dst.updatePixels();
  }

  /**
   * Step 10:
   * Threshold the Y channel from YCbCr (Y in [0..255]) from the original snapshot.
   *
   * @param {p5.Image} src snapshot
   * @param {p5.Image} dst threshold image
   * @param {number} t threshold in [0..255]
   */
  buildYCbCr_Y_Threshold(src, dst, t) {
    src.loadPixels();
    dst.loadPixels();

    const w = src.width;
    const h = src.height;

    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = (x + y * w) * 4;

        const rr = src.pixels[i];
        const gg = src.pixels[i + 1];
        const bb = src.pixels[i + 2];

        const { y: yy } = this.rgbToYCbCr(rr, gg, bb);
        const out = yy > t ? 255 : 0;

        dst.pixels[i] = out;
        dst.pixels[i + 1] = out;
        dst.pixels[i + 2] = out;
        dst.pixels[i + 3] = 255;
      }
    }

    dst.updatePixels();
  }

  // ---------- Step 11: Face detection (objectdetect, Week 19 style) ----------
  /**
   * Step 11:
   * Initialize the frontal face detector using the snapshot resolution.
   */
  initFaceDetector() {
    this.detector = new objectdetect.detector(
      SNAP_W,
      SNAP_H,
      1.1,
      objectdetect.frontalface
    );
  }

  /**
   * Step 11:
   * Run face detection on the snapshot by drawing it into an offscreen canvas.
   */
  detectFaces() {
    if (!this.hasSnapshot || !this.detector) return;

    this.snapshotG.image(this.snapshotImg, 0, 0, SNAP_W, SNAP_H);
    this.faceRects = this.detector.detect(this.snapshotG.canvas) || [];
  }

  /**
   * Select a single face region for replacement using largest area,
   * optionally filtering weak detections by confidence.
   *
   * objectdetect format: [x, y, w, h, confidence]
   *
   * @param {Array} faces
   * @returns {{x:number, y:number, w:number, h:number}|null}
   */
  pickLargestFace(faces) {
    if (!faces || faces.length === 0) return null;

    let best = null;
    let bestArea = -1;

    for (const f of faces) {
      const conf = f[4];
      if (conf !== undefined && conf <= this.minFaceConfidence) continue;

      const area = f[2] * f[3];
      if (area > bestArea) {
        bestArea = area;
        best = { x: f[0], y: f[1], w: f[2], h: f[3] };
      }
    }

    return best;
  }

  // ---------- Step 12: Face replacement helpers ----------
  /**
   * Step 12a:
   * Convert pixels inside the face box to grayscale (manual pixel loop).
   * Constraint: do not use img.filter(GRAY).
   *
   * @param {p5.Image} src original snapshot
   * @param {p5.Image} dst output image (modified in-place)
   * @param {{x:number, y:number, w:number, h:number}} box face region
   */
  applyFaceGreyscale(src, dst, box) {
    const x0 = Math.max(0, Math.floor(box.x));
    const y0 = Math.max(0, Math.floor(box.y));
    const x1 = Math.min(src.width, Math.floor(box.x + box.w));
    const y1 = Math.min(src.height, Math.floor(box.y + box.h));

    src.loadPixels();
    dst.loadPixels();

    for (let y = y0; y < y1; y++) {
      for (let x = x0; x < x1; x++) {
        const i = (x + y * src.width) * 4;

        const rr = src.pixels[i];
        const gg = src.pixels[i + 1];
        const bb = src.pixels[i + 2];

        const gray = (rr + gg + bb) / 3;

        dst.pixels[i] = gray;
        dst.pixels[i + 1] = gray;
        dst.pixels[i + 2] = gray;
        dst.pixels[i + 3] = 255;
      }
    }

    dst.updatePixels();
  }

  /**
   * Step 12b:
   * Flip the face region horizontally using nested loops and pixel indexing.
   * Constraint: do not use scale() or translate().
   *
   * @param {p5.Image} src original snapshot
   * @param {p5.Image} dst output image (modified in-place)
   * @param {{x:number, y:number, w:number, h:number}} box face region
   */
  applyFaceFlipHorizontal(src, dst, box) {
    const x0 = Math.max(0, Math.floor(box.x));
    const y0 = Math.max(0, Math.floor(box.y));
    const x1 = Math.min(src.width, Math.floor(box.x + box.w));
    const y1 = Math.min(src.height, Math.floor(box.y + box.h));
    const w = x1 - x0;

    src.loadPixels();
    dst.loadPixels();

    for (let y = y0; y < y1; y++) {
      for (let x = x0; x < x1; x++) {
        const localX = x - x0;
        const flippedX = x0 + (w - 1 - localX);

        const srcIdx = (flippedX + y * src.width) * 4;
        const dstIdx = (x + y * src.width) * 4;

        dst.pixels[dstIdx] = src.pixels[srcIdx];
        dst.pixels[dstIdx + 1] = src.pixels[srcIdx + 1];
        dst.pixels[dstIdx + 2] = src.pixels[srcIdx + 2];
        dst.pixels[dstIdx + 3] = 255;
      }
    }

    dst.updatePixels();
  }

  /**
   * Step 12c (ii–v):
   * Draw pixelation circles over the face region using 5×5 blocks.
   * The image used for sampling must already be grayscale (Step 12c-i).
   *
   * @param {p5.Image} grayImg grayscale image (for intensity sampling)
   * @param {{x:number, y:number, w:number, h:number}} box face region
   * @param {{x:number, y:number, w:number, h:number}} cellImgRect grid cell image rect
   */
  drawPixelatedFaceOverlay(grayImg, box, cellImgRect) {
    grayImg.loadPixels();

    const blockSize = 5;

    const x0 = Math.max(0, Math.floor(box.x));
    const y0 = Math.max(0, Math.floor(box.y));
    const x1 = Math.min(SNAP_W, Math.floor(box.x + box.w));
    const y1 = Math.min(SNAP_H, Math.floor(box.y + box.h));

    const sx = cellImgRect.w / SNAP_W;
    const sy = cellImgRect.h / SNAP_H;

    push();
    noStroke();

    for (let by = y0; by < y1; by += blockSize) {
      for (let bx = x0; bx < x1; bx += blockSize) {
        const bxEnd = Math.min(bx + blockSize, x1);
        const byEnd = Math.min(by + blockSize, y1);

        let sum = 0;
        let count = 0;

        // Step 12c-iii: average intensity inside block
        for (let y = by; y < byEnd; y++) {
          for (let x = bx; x < bxEnd; x++) {
            const i = (x + y * SNAP_W) * 4;
            sum += grayImg.pixels[i]; // grayscale intensity
            count++;
          }
        }

        const avg = count > 0 ? sum / count : 0;
        fill(avg);

        // Step 12c-iv: filled circle centered in the block
        const cx = (bx + (bxEnd - bx) / 2) * sx;
        const cy = (by + (byEnd - by) / 2) * sy;

        circle(
          cellImgRect.x + cx,
          cellImgRect.y + cy,
          blockSize * sx
        );
      }
    }

    pop();
  }

    /**
   * Extension: Build a binary mask for a neon-green marker/object using HSV.
   * White = detected pixels, Black = background.
   * Uses the existing rgbToHsv() which returns {h179, s255, v255}.
   * @param {p5.Image} src  160x120 live frame (extSrcImg)
   * @param {p5.Image} dst  160x120 mask (extMaskImg)
   */
  buildGreenMask(src, dst) {
    src.loadPixels();
    dst.loadPixels();

    const w = src.width;
    const h = src.height;

    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = (x + y * w) * 4;

        const r = src.pixels[i];
        const g = src.pixels[i + 1];
        const b = src.pixels[i + 2];

        const { h179, s255, v255 } = this.rgbToHsv(r, g, b);

        const isGreen =
          h179 >= this.extHueMin &&
          h179 <= this.extHueMax &&
          s255 >= this.extSatMin &&
          v255 >= this.extValMin;

        const out = isGreen ? 255 : 0;

        dst.pixels[i] = out;
        dst.pixels[i + 1] = out;
        dst.pixels[i + 2] = out;
        dst.pixels[i + 3] = 255;
      }
    }

    dst.updatePixels();
  }

  /**
   * Extension: Compute centroid (average x,y) of white pixels in a binary mask.
   * Returns null if not enough pixels are found (prevents noise/jitter).
   * @param {p5.Image} maskImg
   * @returns {{x:number,y:number}|null}
   */
  findMaskCentroid(maskImg) {
    maskImg.loadPixels();

    const w = maskImg.width;
    const h = maskImg.height;

    let sumX = 0;
    let sumY = 0;
    let count = 0;

    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = (x + y * w) * 4;
        const v = maskImg.pixels[i]; // 0 or 255

        if (v > 0) {
          sumX += x;
          sumY += y;
          count++;
        }
      }
    }

    if (count < this.extMinPixels) return null;

    return { x: sumX / count, y: sumY / count };
  }

  /**
   * Extension: Update drawing layer based on current centroid.
   * Uses smoothing to reduce jitter and prevents long "jump lines".
   * @param {{x:number,y:number}|null} pt
   */
  updateHandDraw(pt) {
    if (!pt) {
      // Lost target: reset previous point to avoid long lines when it reappears.
      this.extPrevPt = null;
      this.extSmooth.has = false;
      return;
    }

    // Exponential smoothing
    const alpha = 0.35; // lower = smoother, higher = more responsive

    if (!this.extSmooth.has) {
      this.extSmooth.x = pt.x;
      this.extSmooth.y = pt.y;
      this.extSmooth.has = true;
    } else {
      this.extSmooth.x = lerp(this.extSmooth.x, pt.x, alpha);
      this.extSmooth.y = lerp(this.extSmooth.y, pt.y, alpha);
    }

    const cur = { x: this.extSmooth.x, y: this.extSmooth.y };

    if (!this.extPrevPt) {
      this.extPrevPt = cur;
      return;
    }

    // Draw / erase on the offscreen 160x120 layer
    if (this.extEraser) {
      this.extLayer.push();
      this.extLayer.erase();
      this.extLayer.noStroke();
      this.extLayer.circle(cur.x, cur.y, 16);
      this.extLayer.noErase();
      this.extLayer.pop();
    } else {
      this.extLayer.push();
      this.extLayer.strokeWeight(this.extStrokeW);
      this.extLayer.stroke(0);          // pen colour (black)
      this.extLayer.noFill();
      this.extLayer.line(this.extPrevPt.x, this.extPrevPt.y, cur.x, cur.y);
      this.extLayer.pop();
    }

    this.extPrevPt = cur;
  }

  // ---------- UI: sliders (Steps 7–8 and 10) ----------
  /**
   * Step 7–8:
   * Create sliders (0..255) for thresholding each RGB channel.
   * Labels are DOM elements placed below the canvas (not inside grid cells).
   */
  initThresholdSliders() {
    const leftX = 10;
    const baseY = CANVAS_H + 10;

    this.rLabel = createDiv("");
    this.gLabel = createDiv("");
    this.bLabel = createDiv("");

    this.rSlider = createSlider(0, 255, this.rThresh, 1);
    this.gSlider = createSlider(0, 255, this.gThresh, 1);
    this.bSlider = createSlider(0, 255, this.bThresh, 1);

    this.rSlider.style("width", "620px");
    this.gSlider.style("width", "620px");
    this.bSlider.style("width", "620px");

    this.rLabel.position(leftX, baseY - 18);
    this.rSlider.position(leftX, baseY);

    this.gLabel.position(leftX, baseY + 12);
    this.gSlider.position(leftX, baseY + 30);

    this.bLabel.position(leftX, baseY + 42);
    this.bSlider.position(leftX, baseY + 60);

    this.rSlider.input(() => { this.rThresh = this.rSlider.value(); });
    this.gSlider.input(() => { this.gThresh = this.gSlider.value(); });
    this.bSlider.input(() => { this.bThresh = this.bSlider.value(); });

    this.updateSliderLabels();
  }

  /**
   * Step 10:
   * Create sliders (0..255) for colour-space thresholding:
   * - HSV Value channel (V)
   * - YCbCr luminance channel (Y)
   */
  initColourSpaceThresholdSliders() {
    const leftX = 10;
    const baseY = CANVAS_H + 100;

    this.hsvVLabel = createDiv("");
    this.ycbcrYLabel = createDiv("");

    this.hsvVSlider = createSlider(0, 255, this.hsvVThresh, 1);
    this.ycbcrYSlider = createSlider(0, 255, this.ycbcrYThresh, 1);

    this.hsvVSlider.style("width", "620px");
    this.ycbcrYSlider.style("width", "620px");

    this.hsvVLabel.position(leftX, baseY - 18);
    this.hsvVSlider.position(leftX, baseY);

    this.ycbcrYLabel.position(leftX, baseY + 12);
    this.ycbcrYSlider.position(leftX, baseY + 30);

    this.hsvVSlider.input(() => { this.hsvVThresh = this.hsvVSlider.value(); });
    this.ycbcrYSlider.input(() => { this.ycbcrYThresh = this.ycbcrYSlider.value(); });

    this.updateSliderLabels();
  }

  /**
   * Keep all slider labels consistent and values visible for assessment.
   */
  updateSliderLabels() {
    if (this.rLabel) this.rLabel.html(`R threshold: ${this.rThresh}`);
    if (this.gLabel) this.gLabel.html(`G threshold: ${this.gThresh}`);
    if (this.bLabel) this.bLabel.html(`B threshold: ${this.bThresh}`);
    if (this.hsvVLabel) this.hsvVLabel.html(`HSV V threshold: ${this.hsvVThresh}`);
    if (this.ycbcrYLabel) this.ycbcrYLabel.html(`YCbCr Y threshold: ${this.ycbcrYThresh}`);
  }

  // ---------- Grid drawing helpers ----------
  drawCellWebcam(col, row, title) {
    const cell = this.grid.cellRect(col, row);
    this.grid.drawCellFrame(cell, title);

    if (!this.webcam.isReady()) {
      this.grid.drawCellMessage(cell, "Waiting for webcam...");
      return;
    }

    image(this.webcam.capture, cell.x, cell.y + this.grid.titleH, cell.w, cell.h - this.grid.titleH);
  }

  drawCellImage(col, row, title, img) {
    const cell = this.grid.cellRect(col, row);
    this.grid.drawCellFrame(cell, title);

    if (!img) {
      this.grid.drawCellMessage(cell, "Press S to take snapshot");
      return;
    }

    image(img, cell.x, cell.y + this.grid.titleH, cell.w, cell.h - this.grid.titleH);
  }

  drawCellGrayMinus20(col, row, title) {
    const cell = this.grid.cellRect(col, row);
    this.grid.drawCellFrame(cell, title);

    if (!this.hasSnapshot) {
      this.grid.drawCellMessage(cell, "Press S to take snapshot");
      return;
    }

    image(this.grayMinus20Img, cell.x, cell.y + this.grid.titleH, cell.w, cell.h - this.grid.titleH);
  }

  drawCellThreshold(col, row, title, srcImg, dstImg, slider) {
    const cell = this.grid.cellRect(col, row);
    this.grid.drawCellFrame(cell, title);

    if (!this.hasSnapshot || !this.rgbReady || !slider) {
      this.grid.drawCellMessage(cell, "Press S to take snapshot");
      return;
    }

    const t = slider.value();
    this.buildThreshold(srcImg, dstImg, t);
    image(dstImg, cell.x, cell.y + this.grid.titleH, cell.w, cell.h - this.grid.titleH);
  }

  drawCellHSV(col, row, title) {
    const cell = this.grid.cellRect(col, row);
    this.grid.drawCellFrame(cell, title);

    if (!this.hasSnapshot) {
      this.grid.drawCellMessage(cell, "Press S to take snapshot");
      return;
    }

    this.buildHSV(this.snapshotImg, this.hsvImg);
    image(this.hsvImg, cell.x, cell.y + this.grid.titleH, cell.w, cell.h - this.grid.titleH);
  }

  drawCellYCbCr(col, row, title) {
    const cell = this.grid.cellRect(col, row);
    this.grid.drawCellFrame(cell, title);

    if (!this.hasSnapshot) {
      this.grid.drawCellMessage(cell, "Press S to take snapshot");
      return;
    }

    this.buildYCbCr(this.snapshotImg, this.ycbcrImg);
    image(this.ycbcrImg, cell.x, cell.y + this.grid.titleH, cell.w, cell.h - this.grid.titleH);
  }

  drawCellHSV_V_Threshold(col, row, title) {
    const cell = this.grid.cellRect(col, row);
    this.grid.drawCellFrame(cell, title);

    if (!this.hasSnapshot || !this.hsvVSlider) {
      this.grid.drawCellMessage(cell, "Press S to take snapshot");
      return;
    }

    const t = this.hsvVSlider.value();
    this.buildHSV_V_Threshold(this.snapshotImg, this.hsvThreshImg, t);
    image(this.hsvThreshImg, cell.x, cell.y + this.grid.titleH, cell.w, cell.h - this.grid.titleH);
  }

  drawCellYCbCr_Y_Threshold(col, row, title) {
    const cell = this.grid.cellRect(col, row);
    this.grid.drawCellFrame(cell, title);

    if (!this.hasSnapshot || !this.ycbcrYSlider) {
      this.grid.drawCellMessage(cell, "Press S to take snapshot");
      return;
    }

    const t = this.ycbcrYSlider.value();
    this.buildYCbCr_Y_Threshold(this.snapshotImg, this.ycbcrThreshImg, t);
    image(this.ycbcrThreshImg, cell.x, cell.y + this.grid.titleH, cell.w, cell.h - this.grid.titleH);
  }

    /**
   * Extension: Independent cell showing live webcam + drawing overlay.
   * Does not affect the coursework grid layout.
   */
  drawCellHandDraw(col, row, title) {
    const cell = this.grid.cellRect(col, row);
    this.grid.drawCellFrame(cell, title);

    if (!this.webcam.isReady()) {
      this.grid.drawCellMessage(cell, "Waiting for webcam...");
      return;
    }

    // 160x120 live frame for consistent processing
    this.extSrcImg.copy(
      this.webcam.capture,
      0, 0, this.webcam.capture.width, this.webcam.capture.height,
      0, 0, SNAP_W, SNAP_H
    );

    // Update drawing only when enabled
    if (this.extEnabled) {
      this.buildGreenMask(this.extSrcImg, this.extMaskImg);
      const pt = this.findMaskCentroid(this.extMaskImg);
      this.updateHandDraw(pt);
    }

    // Render: live frame (scaled to cell) + overlay drawing layer
    image(
      this.extSrcImg,
      cell.x,
      cell.y + this.grid.titleH,
      cell.w,
      cell.h - this.grid.titleH
    );

    image(
      this.extLayer,
      cell.x,
      cell.y + this.grid.titleH,
      cell.w,
      cell.h - this.grid.titleH
    );

    //small status text inside title bar area
    push();
    fill(20);
    textSize(10);
    textAlign(RIGHT, CENTER);
    const status = this.extEnabled ? (this.extEraser ? "ERASE" : "DRAW") : "OFF";
    text(status, cell.x + cell.w - 6, cell.y + this.grid.titleH / 2);
    pop();
  }

  // ---------- Step 11–12: Face detection + replacement cell ----------
  /**
   * Step 11–12:
   * Draw snapshot in the cell, then either:
   * - Step 11 (mode 0): draw the detected face bounding box, or
   * - Step 12 (mode 1/2/3): replace the face region using the selected transformation.
   */
  drawCellFaceDetectionAndReplacement(col, row, title) {
    const cell = this.grid.cellRect(col, row);
    this.grid.drawCellFrame(cell, title);

    if (!this.hasSnapshot) {
      this.grid.drawCellMessage(cell, "Press S to take snapshot");
      return;
    }

    const imgRect = {
      x: cell.x,
      y: cell.y + this.grid.titleH,
      w: cell.w,
      h: cell.h - this.grid.titleH
    };

    // Draw the base snapshot
    image(this.snapshotImg, imgRect.x, imgRect.y, imgRect.w, imgRect.h);

    const faceBox = this.pickLargestFace(this.faceRects);
    if (!faceBox) return;

    // Step 11: show bounding box only
    if (this.faceMode === 0) {
      const sx = imgRect.w / SNAP_W;
      const sy = imgRect.h / SNAP_H;

      push();
      noFill();
      stroke(255, 0, 0);
      strokeWeight(2);
      rect(
        imgRect.x + faceBox.x * sx,
        imgRect.y + faceBox.y * sy,
        faceBox.w * sx,
        faceBox.h * sy
      );
      pop();
      return;
    }

    // Step 12: build replacement image into reusable work buffer
    this.faceWorkImg.copy(this.snapshotImg, 0, 0, SNAP_W, SNAP_H, 0, 0, SNAP_W, SNAP_H);

    // Step 12a: grayscale face region
    if (this.faceMode === 1) {
      this.applyFaceGreyscale(this.snapshotImg, this.faceWorkImg, faceBox);
      image(this.faceWorkImg, imgRect.x, imgRect.y, imgRect.w, imgRect.h);
      return;
    }

    // Step 12b: flipped face region
    if (this.faceMode === 2) {
      this.applyFaceFlipHorizontal(this.snapshotImg, this.faceWorkImg, faceBox);
      image(this.faceWorkImg, imgRect.x, imgRect.y, imgRect.w, imgRect.h);
      return;
    }

    // Step 12c: pixelated face region (grayscale + 5×5 blocks + average + circles)
    if (this.faceMode === 3) {
      // Step 12c-i: grayscale face region first
      this.applyFaceGreyscale(this.snapshotImg, this.faceWorkImg, faceBox);

      // Draw grayscale base
      image(this.faceWorkImg, imgRect.x, imgRect.y, imgRect.w, imgRect.h);

      // Step 12c-ii..v: pixelation overlay
      this.drawPixelatedFaceOverlay(this.faceWorkImg, faceBox, imgRect);
    }
  }

  // ---------- App main draw ----------
  /**
   * Draw the full grid each frame.
   * This method only draws; processing outputs are computed from snapshot buffers.
   */
  draw() {
    background(245);

    // Keep slider labels up-to-date
    this.updateSliderLabels();

    // Row 1
    this.drawCellWebcam(0, 0, "Webcam image");
    this.drawCellGrayMinus20(1, 0, "Grayscale, brightness -20%");
    this.drawCellHandDraw(2, 0, "Extension: Hand Draw");

    // Row 2
    this.drawCellImage(0, 1, "Red channel", this.rgbReady ? this.rImg : null);
    this.drawCellImage(1, 1, "Green channel", this.rgbReady ? this.gImg : null);
    this.drawCellImage(2, 1, "Blue channel", this.rgbReady ? this.bImg : null);

    // Row 3
    this.drawCellThreshold(0, 2, "Threshold image", this.rImg, this.rThreshImg, this.rSlider);
    this.drawCellThreshold(1, 2, "Threshold image", this.gImg, this.gThreshImg, this.gSlider);
    this.drawCellThreshold(2, 2, "Threshold image", this.bImg, this.bThreshImg, this.bSlider);

    // Row 4
    this.drawCellWebcam(0, 3, "Webcam image");
    this.drawCellHSV(1, 3, "Colour space 1 (HSV)");
    this.drawCellYCbCr(2, 3, "Colour space 2 (YCbCr)");

    // Row 5
    this.drawCellFaceDetectionAndReplacement(0, 4, "Face detection and replaced face images");
    this.drawCellHSV_V_Threshold(1, 4, "Threshold image from colour space 1 (HSV V)");
    this.drawCellYCbCr_Y_Threshold(2, 4, "Threshold image from colour space 2 (YCbCr Y)");
  }
}

// ---------- Webcam source (module-style capture wrapper) ----------
class WebcamSource {
  constructor() {
    this.capture = null;
  }

  init() {
    pixelDensity(1);
    this.capture = createCapture(VIDEO);
    this.capture.size(CANVAS_W, CANVAS_H);
    this.capture.hide();
  }

  isReady() {
    return this.capture !== null;
  }
}

// ---------- Grid layout helper (consistent titles + frames) ----------
class GridLayout {
  constructor(cols, rows, canvasW, canvasH) {
    this.cols = cols;
    this.rows = rows;

    this.canvasW = canvasW;
    this.canvasH = canvasH;

    this.cellW = Math.floor(canvasW / cols);
    this.cellH = Math.floor(canvasH / rows);

    this.titleH = 16;
  }

  cellRect(col, row) {
    return {
      x: col * this.cellW,
      y: row * this.cellH,
      w: this.cellW,
      h: this.cellH
    };
  }

  drawCellFrame(cell, title) {
    // Outer border
    stroke(180);
    noFill();
    rectMode(CORNER);
    rect(cell.x, cell.y, cell.w, cell.h);

    // Title bar
    noStroke();
    fill(230);
    rect(cell.x, cell.y, cell.w, this.titleH);

    // Title text
    fill(20);
    textSize(11);
    textAlign(LEFT, CENTER);
    text(title, cell.x + 6, cell.y + this.titleH / 2);
  }

  drawCellMessage(cell, msg) {
    fill(90);
    textSize(12);
    textAlign(CENTER, CENTER);
    text(msg, cell.x + cell.w / 2, cell.y + cell.h / 2);
  }
}

// ---------- p5.js lifecycle ----------
function setup() {
  createCanvas(CANVAS_W, CANVAS_H);
  app = new App();
  app.init();
}

function draw() {
  app.draw();
}

function keyPressed() {
  if (key === "s" || key === "S") {
    app.takeSnapshot();
    return;
  }

  if (key === "1") { app.faceMode = 1; return; }
  if (key === "2") { app.faceMode = 2; return; }
  if (key === "3") { app.faceMode = 3; return; }

  // Extension controls
  if (key === "d" || key === "D") {
    app.extEnabled = !app.extEnabled;
    // reset tracking to avoid a jump line when toggling
    app.extPrevPt = null;
    app.extSmooth.has = false;
    return;
  }

  if (key === "e" || key === "E") {
    app.extEraser = !app.extEraser;
    return;
  }

  if (key === "c" || key === "C") {
    app.extLayer.clear();
    app.extPrevPt = null;
    app.extSmooth.has = false;
    return;
  }
}