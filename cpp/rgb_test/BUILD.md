# rgb_test -- D456 RGB stream viewer

Streams the D456 colour camera to a live window. Press `q` to quit.

## 1. Install OpenCV (one-time)

Download the Windows installer from https://opencv.org/releases/ (e.g. 4.10.0).
Run it and install to `C:\opencv`.

## 2. Configure

```powershell
cmake -S . -B build -DOpenCV_DIR="C:/opencv/build"
```

If your RealSense SDK or OpenCV is in a non-default location, add it to CMAKE_PREFIX_PATH:

```powershell
cmake -S . -B build `
  -DOpenCV_DIR="C:/opencv/build" `
  -DCMAKE_PREFIX_PATH="C:/Program Files (x86)/Intel RealSense SDK 2.0"
```

## 3. Build

```powershell
cmake --build build
```

## 4. Copy DLLs (one-time, if not already in PATH)

```powershell
Copy-Item "C:\Program Files (x86)\Intel RealSense SDK 2.0\bin\x64\realsense2.dll" .\build\Debug\
Copy-Item "C:\opencv\build\x64\vc16\bin\opencv_world4100.dll" .\build\Debug\
```

Adjust `opencv_world4100.dll` to match your installed OpenCV version number.

## 5. Run

```powershell
.\build\Debug\rgb_test.exe
```

## Clean

```powershell
Remove-Item -Recurse -Force build
```
