# 3D CAD Models & Manufacturing Files

> **WRO 2026 Future Engineers — Team Durnibar**

This folder contains all 3D CAD design files, assemblies, and manufacturing models for **Durnibar 2.0**.

---

## 📂 Subdirectories

- **[`cad_source/`](./cad_source/)**: Fusion 360 source files (`Durnibar 2.0.f3z` robot model, `Gear Box v2.0.f3d` 2-stage transmission).
- **[`3d_print_files/`](./3d_print_files/)**: Printable STL / STEP files for chassis components, sensor brackets, gear sets, and mounts.

---

## 🖨 Recommended 3D Printing Settings

- **Material**: PETG or ABS (for impact resistance during wall contact)
- **Layer Height**: 0.20 mm
- **Infill**: 40% Gyroid infill for high strength-to-weight ratio
- **Perimeters**: 4 walls / 4 top & bottom layers

---

## 🔍 Viewing the 3D CAD Files

To view and interact with the 3D design models (such as `.f3d`, `.f3z`, and `.stl` files) in this repository:

### Method 1: Visual Studio Code (Recommended)
This repository includes a VS Code configuration that recommends CAD/3D viewer extensions:
1. Open this repository folder in **VS Code**.
2. You will be prompted to install the recommended extensions. Click **Install**.
3. Alternatively, open the Extensions marketplace (`Ctrl+Shift+X`) and install:
   * **CAD Viewer** (`thingraph.cad-viewer`)
   * **STL Viewer** (`mike-lischke.vscode-stl-viewer`)
4. Simply double-click any `.stl`, `.step`, or `.gltf` file to open and orbit the model interactively.

### Method 2: Autodesk Viewer (Free & Web-Based)
Since `.f3d` and `.f3z` are proprietary Autodesk Fusion 360 files:
1. Open the free online [Autodesk Viewer](https://viewer.autodesk.com/).
2. Upload the file (e.g., [`Durnibar 2.0.f3z`](./cad_source/Durnibar%202.0.f3z) or [`Gear Box v2.0.f3d`](./cad_source/Gear%20Box%20v2.0.f3d)).
3. This allows you to explore assemblies, isolate parts, measure dimensions, and view the full 3D assembly structure in your browser.

### Method 3: GitHub 3D Viewer (For STL models)
When browsing this repository on GitHub.com, simply click on any `.stl` model file. GitHub has a built-in WebGL interactive viewer that allows you to rotate, pan, and zoom the model directly in your browser.
