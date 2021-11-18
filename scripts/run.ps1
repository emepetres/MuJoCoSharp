pushd $PSScriptRoot\..

if (-Not (Test-Path -Path '.\build\mujoco210'))
{
    mkdir -F build
    pushd build
    Invoke-WebRequest -Uri https://mujoco.org/download/mujoco210-windows-x86_64.zip -OutFile ./mujoco210.tar.gz
    tar -xf mujoco210.tar.gz
    rm mujoco210.tar.gz
    popd
}

dotnet run -p src\BindingGenerator

popd
