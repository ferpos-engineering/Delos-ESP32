using System.Windows;
using Esp32FlasherUI.ViewModels;

namespace Esp32FlasherUI;

public partial class MainWindow : Window
{
    public MainWindow()
    {
        InitializeComponent();
        DataContext = new MainViewModel();
    }
}
