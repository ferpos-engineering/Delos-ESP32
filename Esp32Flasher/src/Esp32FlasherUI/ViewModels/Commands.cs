using System.Windows.Input;

namespace Esp32FlasherUI.ViewModels;

public sealed class RelayCommand : ICommand
{
    private readonly Action<object?> _exec;
    private readonly Func<object?, bool>? _can;
    public event EventHandler? CanExecuteChanged;

    public RelayCommand(Action<object?> exec, Func<object?, bool>? can = null)
    {
        _exec = exec;
        _can = can;
    }

    public bool CanExecute(object? parameter) => _can?.Invoke(parameter) ?? true;
    public void Execute(object? parameter) => _exec(parameter);
    public void RaiseCanExecuteChanged() => CanExecuteChanged?.Invoke(this, EventArgs.Empty);
}

public sealed class AsyncRelayCommand : ICommand
{
    private readonly Func<object?, Task> _exec;
    private readonly Func<object?, bool>? _can;
    private bool _running;

    public event EventHandler? CanExecuteChanged;

    public AsyncRelayCommand(Func<object?, Task> exec, Func<object?, bool>? can = null)
    {
        _exec = exec;
        _can = can;
    }

    public bool CanExecute(object? parameter) => !_running && (_can?.Invoke(parameter) ?? true);

    public async void Execute(object? parameter)
    {
        if (!CanExecute(parameter)) return;
        _running = true;
        CanExecuteChanged?.Invoke(this, EventArgs.Empty);

        try
        {
            await _exec(parameter);
        }
        finally
        {
            _running = false;
            CanExecuteChanged?.Invoke(this, EventArgs.Empty);
        }
    }
}
