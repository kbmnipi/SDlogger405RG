$base = 'C:\Users\ETS80\.platformio\packages\framework-stm32cubef4\Middlewares'
Write-Output "=== FatFs tree ==="
Get-ChildItem -Path $base -Recurse -ErrorAction SilentlyContinue | Where-Object { $_.FullName -match 'FatFs|fatfs' } | Select-Object -ExpandProperty FullName

