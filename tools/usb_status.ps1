param([switch]$Json)

# Read-only: does not open a COM port, change drivers, or send motor commands.
$ErrorActionPreference = 'Stop'
$devices = @(Get-PnpDevice -PresentOnly | Where-Object {
    $_.InstanceId -like 'USB\VID_CAFE&PID_4001*'
})

function Read-DeviceProperty([string]$InstanceId, [string]$Key) {
    $property = Get-PnpDeviceProperty -InstanceId $InstanceId -KeyName $Key -ErrorAction SilentlyContinue
    if ($null -ne $property) { return $property.Data }
    return $null
}

$records = @(foreach ($device in $devices) {
    [pscustomobject]@{
        Status = [string]$device.Status
        Class = [string]$device.Class
        Name = [string]$device.FriendlyName
        InstanceId = [string]$device.InstanceId
        Service = Read-DeviceProperty $device.InstanceId 'DEVPKEY_Device_Service'
        DriverInf = Read-DeviceProperty $device.InstanceId 'DEVPKEY_Device_DriverInfPath'
        MatchingId = Read-DeviceProperty $device.InstanceId 'DEVPKEY_Device_MatchingDeviceId'
    }
})
$serialInterfaces = @($records | Where-Object { $_.Class -eq 'Ports' -and $_.Service -eq 'usbser' })
if ($Json) {
    [pscustomobject]@{
        Present = $records.Count -gt 0
        SerialDriverReady = $serialInterfaces.Count -gt 0
        Devices = $records
    } | ConvertTo-Json -Depth 5
} else {
    if ($records.Count -eq 0) {
        Write-Output 'F411 USB device CAFE:4001 is not currently present.'
    } else {
        $records | Format-List
        if ($serialInterfaces.Count -gt 0) {
            Write-Output 'USB serial driver is bound. Use the COM number shown above; data transfer is not tested here.'
        } else {
            Write-Output 'No usbser COM interface found. See the Windows driver section in USB_CDC_GUIDE.md.'
        }
    }
}
if ($serialInterfaces.Count -gt 0) { exit 0 }
if ($records.Count -eq 0) { exit 2 }
exit 1
