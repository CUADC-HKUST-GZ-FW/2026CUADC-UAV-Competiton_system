[CmdletBinding()]
param(
    [string]$JetsonHost = "192.168.55.2",
    [string]$JetsonUser = "nx164"
)

$ErrorActionPreference = "Stop"

function Invoke-Git {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Arguments)

    & git @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "git $($Arguments -join ' ') failed with exit code $LASTEXITCODE"
    }
}

$repoRoot = (& git rev-parse --show-toplevel 2>$null).Trim()
if ($LASTEXITCODE -ne 0 -or -not $repoRoot) {
    throw "Run this script from the competition-system Git repository."
}

Push-Location $repoRoot
$bundle = Join-Path ([System.IO.Path]::GetTempPath()) "cuadc-main-$([guid]::NewGuid().ToString('N')).bundle"
$remoteBundle = "/home/$JetsonUser/cuadc-main-from-laptop.bundle"

try {
    $branch = (& git branch --show-current).Trim()
    if ($branch -ne "main") {
        throw "Local repository must be on main; current branch is '$branch'."
    }

    $trackedChanges = & git status --porcelain --untracked-files=no
    if ($LASTEXITCODE -ne 0 -or $trackedChanges) {
        throw "Local tracked files are not clean. Commit them before deployment."
    }

    Write-Host "[INFO] Fetching and fast-forwarding local main..."
    Invoke-Git fetch origin main
    Invoke-Git merge --ff-only origin/main

    $commit = (& git rev-parse HEAD).Trim()
    Write-Host "[INFO] Packaging commit $commit..."
    Invoke-Git bundle create $bundle main

    Write-Host "[INFO] Copying Git bundle to $JetsonUser@$JetsonHost..."
    & scp $bundle "${JetsonUser}@${JetsonHost}:$remoteBundle"
    if ($LASTEXITCODE -ne 0) {
        throw "scp failed with exit code $LASTEXITCODE"
    }

    $remoteScript = @'
set -Eeuo pipefail

repo="$HOME/2026CUADC-UAV-Competiton_system"
bundle="$HOME/cuadc-main-from-laptop.bundle"

[[ -d "$repo/.git" ]] || {
    echo "[ERROR] repository clone not found: $repo" >&2
    exit 1
}
[[ -f "$bundle" ]] || {
    echo "[ERROR] transferred Git bundle not found: $bundle" >&2
    exit 1
}

cd "$repo"
[[ "$(git branch --show-current)" == "main" ]] || {
    echo "[ERROR] NX164 deployment clone is not on main." >&2
    exit 1
}
[[ -z "$(git status --porcelain --untracked-files=no)" ]] || {
    echo "[ERROR] NX164 deployment clone has tracked local changes." >&2
    exit 1
}

if systemctl is-active --quiet youth-vision; then
    echo "[ERROR] youth-vision is active; stop it before deployment." >&2
    exit 1
fi

git fetch "$bundle" main:refs/remotes/laptop/main
git merge --ff-only refs/remotes/laptop/main
git update-ref refs/remotes/origin/main refs/remotes/laptop/main
rm -f -- "$bundle"

./deploy/update_jetson.sh --apply-local
git status --short --branch
git log -1 --oneline --decorate
'@

    Write-Host "[INFO] Updating and building NX164..."
    $remoteScript | & ssh "${JetsonUser}@${JetsonHost}" "tr -d '\r' | bash -s"
    if ($LASTEXITCODE -ne 0) {
        throw "NX164 deployment failed with exit code $LASTEXITCODE"
    }

    Write-Host "[OK] NX164 is built from commit $commit. No service was started."
}
finally {
    Pop-Location
    Remove-Item -LiteralPath $bundle -Force -ErrorAction SilentlyContinue
}
