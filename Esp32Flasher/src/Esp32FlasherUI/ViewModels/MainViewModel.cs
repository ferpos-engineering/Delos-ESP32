using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Text;
using System.Windows.Input;
using Microsoft.Win32;
using Esp32FlasherUI.Services;
using System.IO;

namespace Esp32FlasherUI.ViewModels;

public sealed class MainViewModel : INotifyPropertyChanged
{
    public event PropertyChangedEventHandler? PropertyChanged;

    private readonly EsptoolRunner _runner = new();
    private CancellationTokenSource? _cts;

    public ObservableCollection<SerialPortInfo> Ports { get; } = new();
    public ObservableCollection<int> BaudRates { get; } = new() { 115200, 230400, 460800, 921600 };

    private SerialPortInfo? _selectedPort;
    public SerialPortInfo? SelectedPort { get => _selectedPort; set { _selectedPort = value; OnChanged(); } }

    private int _selectedBaud = 460800;
    public int SelectedBaud { get => _selectedBaud; set { _selectedBaud = value; OnChanged(); } }

    public string BootloaderPath { get => _boot; set { _boot = value; OnChanged(); } }
    public string PartitionPath { get => _part; set { _part = value; OnChanged(); } }
    public string FirmwarePath { get => _firmware; set { _firmware = value; OnChanged(); } }

    private string _boot = "";
    private string _part = "";
    private string _firmware = "";

    private int _progress;
    public int Progress { get => _progress; set { _progress = value; OnChanged(); } }

    private string _status = "Idle";
    public string StatusText { get => _status; set { _status = value; OnChanged(); } }

    private bool _isBusy;
    public bool IsBusy { get => _isBusy; set { _isBusy = value; OnChanged(); OnChanged(nameof(IsIdle)); } }
    public bool IsIdle => !IsBusy;

    private readonly StringBuilder _log = new();
    public string LogText => _log.ToString();

    public ICommand RefreshPortsCommand { get; }
    public ICommand BrowseBootloaderCommand { get; }
    public ICommand BrowsePartitionCommand { get; }
    public ICommand BrowseFirmwareCommand { get; }
    public ICommand EraseCommand { get; }
    public ICommand FlashCommand { get; }
    public ICommand CancelCommand { get; }

    public MainViewModel()
    {
        RefreshPortsCommand = new RelayCommand(_ => RefreshPorts());

        BrowseBootloaderCommand = new RelayCommand(_ => BootloaderPath = PickBin());
        BrowsePartitionCommand = new RelayCommand(_ => PartitionPath = PickBin());
        BrowseFirmwareCommand = new RelayCommand(_ => FirmwarePath = PickBin());

        EraseCommand = new AsyncRelayCommand(_ => EraseAsync(), _ => IsIdle);
        FlashCommand = new AsyncRelayCommand(_ => FlashAsync(), _ => IsIdle);
        CancelCommand = new RelayCommand(_ => Cancel(), _ => IsBusy);

        _runner.OutputReceived += (_, line) => AppendLog(line);
        _runner.ProgressChanged += (_, pct) => Progress = pct;
        _runner.StateChanged += (_, st) =>
        {
            IsBusy = st == RunnerState.Running;
            if (!IsBusy) StatusText = "Idle";
        };

        RefreshPorts();
    }

    private void RefreshPorts()
    {
        Ports.Clear();

        var ports = SerialPortService.GetPorts();
        foreach (var port in ports)
            Ports.Add(port);

        SelectedPort ??= Ports.FirstOrDefault();
    }

    private string PickBin()
    {
        var dlg = new OpenFileDialog { Filter = "Binary (*.bin)|*.bin|All files (*.*)|*.*" };
        return dlg.ShowDialog() == true ? dlg.FileName : "";
    }

    private void Cancel()
    {
        _cts?.Cancel();
        _runner.Cancel();
        StatusText = "Cancelling...";
    }

    private async Task EraseAsync()
    {
        if (!ValidateCommon()) return;
        Progress = 0;
        StatusText = "Erasing...";
        AppendLog("[CMD] erase-flash");
        await Run(BuildEraseArgs());
    }

    private async Task FlashAsync()
    {
        if (!ValidateCommon()) return;

        if (!File.Exists(BootloaderPath) || !File.Exists(PartitionPath) || !File.Exists(FirmwarePath))
        {
            StatusText = "Missing .bin file(s)";
            AppendLog("[ERROR] Missing .bin file(s)");
            return;
        }

        Progress = 0;
        StatusText = "Flashing...";
        AppendLog("[CMD] write-flash");
        await Run(BuildFlashArgs());
    }

    private async Task Run(string args)
    {
        _cts = new CancellationTokenSource();
        try
        {
            var exit = await _runner.RunAsync(EsptoolCmd, args, _cts.Token);
            AppendLog($"[EXIT] {exit}");
            StatusText = exit == 0 ? "Done" : $"Failed (exit {exit})";
        }
        catch (OperationCanceledException)
        {
            AppendLog("[CANCELLED]");
            StatusText = "Cancelled";
        }
        catch (Exception ex)
        {
            AppendLog("[ERROR] " + ex);
            StatusText = "Error";
        }
        finally
        {
            AppendLog(Environment.NewLine);
            _cts?.Dispose();
            _cts = null;
        }
    }

    private bool ValidateCommon()
    {
        if (string.IsNullOrWhiteSpace(SelectedPort?.PortName))
        {
            StatusText = "Select COM port";
            return false;
        }

        if (!EsptoolCmd.Exists)
        {
            StatusText = "esptool.cmd not found";
            AppendLog("[ERROR] esptool.cmd not found at:");
            AppendLog(EsptoolCmd.FullName);
            return false;
        }

        return true;
    }

    private FileInfo EsptoolCmd = new FileInfo(Path.Combine(AppContext.BaseDirectory, "esptool.cmd")) ;

    private string BuildEraseArgs()
        => $"--chip esp32c6 -p {SelectedPort?.PortName} -b {SelectedBaud} --before default-reset --after hard-reset erase-flash";

    private string BuildFlashArgs()
        => $"--chip esp32c6 -p {SelectedPort?.PortName} -b {SelectedBaud} --before default-reset --after hard-reset " +
           $"write-flash --flash-mode dio --flash-freq 80m --flash-size detect " +
           $"0x0 \"{BootloaderPath}\" 0x8000 \"{PartitionPath}\" 0x10000 \"{FirmwarePath}\"";

    private void AppendLog(string line)
    {
        // Ensure UI-thread update
        System.Windows.Application.Current.Dispatcher.Invoke(() =>
        {
            _log.AppendLine(line);
            OnChanged(nameof(LogText));
        });
    }

    private void OnChanged([CallerMemberName] string? name = null)
        => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
}
