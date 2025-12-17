# Secure Vercel Deployment Script
# Reads backend/.env and deploys to Vercel with environment variables

Write-Host "`n[INFO] Starting Secure Vercel Deployment...`n" -ForegroundColor Green

# Check if Vercel CLI is installed
try {
    $vercelVersion = vercel --version
    Write-Host "[OK] Vercel CLI installed: $vercelVersion`n" -ForegroundColor Cyan
} catch {
    Write-Host "[ERROR] Vercel CLI not found. Installing...`n" -ForegroundColor Red
    npm install -g vercel
}

# Read .env file
Write-Host "[INFO] Reading environment variables from backend\.env...`n" -ForegroundColor Yellow

$envFile = Get-Content backend\.env -ErrorAction Stop
$envVars = @{}

foreach ($line in $envFile) {
    # Skip comments and empty lines
    if ($line -match "^\s*#" -or $line -match "^\s*$") {
        continue
    }

    # Parse key=value pairs
    if ($line -match "^([^=]+)=(.+)$") {
        $key = $matches[1].Trim()
        $value = $matches[2].Trim()
        $envVars[$key] = $value
    }
}

Write-Host "[OK] Loaded $($envVars.Count) environment variables`n" -ForegroundColor Cyan

# Link to Vercel project
Write-Host "[INFO] Linking to Vercel project...`n" -ForegroundColor Yellow
vercel link --yes

# Deploy critical variables to Vercel
$criticalVars = @(
    "DATABASE_URL",
    "GEMINI_API_KEY",
    "JWT_SECRET_KEY",
    "QDRANT_URL",
    "QDRANT_API_KEY",
    "QDRANT_COLLECTION",
    "EMBEDDING_MODEL",
    "EMBEDDING_DIMENSION",
    "GEMINI_MODEL"
)

Write-Host "`n[INFO] Setting environment variables in Vercel...`n" -ForegroundColor Yellow

foreach ($var in $criticalVars) {
    if ($envVars.ContainsKey($var)) {
        Write-Host "  - Setting $var... " -NoNewline -ForegroundColor Cyan

        # Use echo to pipe value to vercel env add (non-interactive)
        $value = $envVars[$var]
        $output = echo $value | vercel env add $var production 2>&1

        if ($LASTEXITCODE -eq 0) {
            Write-Host "OK" -ForegroundColor Green
        } else {
            Write-Host "SKIPPED (may already exist)" -ForegroundColor Yellow
        }
    } else {
        Write-Host "  - WARNING: $var not found in .env" -ForegroundColor Red
    }
}

# Deploy to production
Write-Host "`n[INFO] Deploying to Vercel Production...`n" -ForegroundColor Green
Write-Host "=========================================`n" -ForegroundColor Cyan

vercel --prod

Write-Host "`n=========================================`n" -ForegroundColor Cyan
Write-Host "[SUCCESS] Deployment complete!`n" -ForegroundColor Green
Write-Host "Check the URL above to access your deployed application.`n" -ForegroundColor Yellow

pause
