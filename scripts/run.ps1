Push-Location $PSScriptRoot\..

if (-Not (Test-Path -Path '.\lib\mujoco210'))
{
    mkdir -F lib
    Push-Location lib
    Invoke-WebRequest -Uri https://mujoco.org/download/mujoco210-windows-x86_64.zip -OutFile ./mujoco210.tar.gz
    tar -xf mujoco210.tar.gz
    Remove-Item mujoco210.tar.gz
    Pop-Location
}

dotnet run --project src\BindingGenerator

Pop-Location
