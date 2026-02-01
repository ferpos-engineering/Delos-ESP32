# Esp32Flasher (WPF, .NET 10)

Simple WPF UI to flash ESP32-C6 using `esptool` via an external wrapper (`esptool.cmd`).

## Expected install layout at runtime

The app expects:

- `Tools\esptool\esptool.cmd`
- `Tools\esptool\python\python.exe` + vendored `site-packages` (esptool and deps)

Relative to the application folder (AppContext.BaseDirectory).

## Publish self-contained (win-x64)

```powershell
dotnet publish src/Esp32FlasherUI -c Release -r win-x64 --self-contained true
```

Single-file:

```powershell
dotnet publish src\Esp32FlasherUI -c Release -r win-x64 --self-contained true `
  /p:PublishSingleFile=true /p:IncludeNativeLibrariesForSelfExtract=true
```

## Notes

- Trimming is disabled by default (PublishTrimmed=false) to avoid WPF/reflection issues.
- esptool progress parsing is best-effort: it looks for "(NN %)" in output lines.
