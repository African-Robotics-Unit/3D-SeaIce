Remove-Item -Recurse -Force build -ErrorAction SilentlyContinue
Write-Host "Build directory removed. Run: cmake -S . -B build && cmake --build build"
