using System.Diagnostics;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;

namespace Esp32FlasherUI.Services;

public enum RunnerState { Idle, Running }

public sealed class EsptoolRunner
{
    public event EventHandler<string>? OutputReceived;
    public event EventHandler<int>? ProgressChanged;
    public event EventHandler<RunnerState>? StateChanged;

    private Process? _proc;
    private static readonly Regex PctRegex = new(@"\((\d{1,3})\s*%\)", RegexOptions.Compiled);

    public bool IsRunning => _proc is { HasExited: false };

    public async Task<int> RunAsync(FileInfo espToolCmd, string args, CancellationToken ct)
    {
        if (!espToolCmd.Exists)
            throw new FileNotFoundException("esptool.cmd not found", espToolCmd.FullName);

        if (IsRunning)
            throw new InvalidOperationException("esptool is already running");

        StateChanged?.Invoke(this, RunnerState.Running);

        var psi = new ProcessStartInfo
        {
            FileName = espToolCmd.FullName,
            Arguments = args,
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true,
            StandardOutputEncoding = Encoding.UTF8,
            StandardErrorEncoding = Encoding.UTF8
        };

        _proc = new Process { StartInfo = psi, EnableRaisingEvents = true };
        _proc.Start();

        var readStdOut = Task.Run(async () =>
        {
            while (!_proc.HasExited)
            {
                ct.ThrowIfCancellationRequested();
                var line = await _proc.StandardOutput.ReadLineAsync();
                if (line is null) break;
                HandleLine(line);
            }
        }, ct);

        var readStdErr = Task.Run(async () =>
        {
            while (!_proc.HasExited)
            {
                ct.ThrowIfCancellationRequested();
                var line = await _proc.StandardError.ReadLineAsync();
                if (line is null) break;
                HandleLine(line);
            }
        }, ct);

        using var reg = ct.Register(() =>
        {
            try
            {
                if (_proc is { HasExited: false })
                    _proc.Kill(entireProcessTree: true);
            }
            catch { /* ignore */ }
        });

        try
        {
            await Task.WhenAll(readStdOut, readStdErr);
        }
        catch (OperationCanceledException)
        {
            // cancellation handled by killing the process
        }

        _proc.WaitForExit();
        var code = _proc.ExitCode;

        _proc.Dispose();
        _proc = null;

        StateChanged?.Invoke(this, RunnerState.Idle);
        return code;
    }

    public void Cancel()
    {
        try
        {
            if (_proc is { HasExited: false })
                _proc.Kill(entireProcessTree: true);
        }
        catch { /* ignore */ }
    }

    private void HandleLine(string line)
    {
        OutputReceived?.Invoke(this, line);

        var m = PctRegex.Match(line);
        if (m.Success && int.TryParse(m.Groups[1].Value, out var pct))
        {
            pct = Math.Clamp(pct, 0, 100);
            ProgressChanged?.Invoke(this, pct);
        }
    }
}
