using System.Management;

namespace Esp32FlasherUI.Services;

public sealed class SerialPortInfo
{
    public string PortName { get; init; } = "";
    public string Description { get; init; } = "";

    public override string ToString()
        => $"{PortName} – {Description}";
}

public static class SerialPortService
{
    public static IReadOnlyList<SerialPortInfo> GetPorts()
    {
        var result = new List<SerialPortInfo>();

        using var searcher = new ManagementObjectSearcher(
            "SELECT * FROM Win32_PnPEntity WHERE Name LIKE '%(COM%'");

        foreach (var obj in searcher.Get())
        {
            var name = obj["Name"]?.ToString();
            if (string.IsNullOrWhiteSpace(name))
                continue;

            int i = name.LastIndexOf("(COM");
            if (i < 0)
                continue;

            string port = name.Substring(i + 1).TrimEnd(')');

            result.Add(new SerialPortInfo
            {
                PortName = port,
                Description = name
            });
        }

        return result
            .OrderBy(p => p.PortName)
            .ToList();
    }
}
