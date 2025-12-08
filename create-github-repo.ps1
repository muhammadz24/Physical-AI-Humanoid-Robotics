# Automated GitHub Repository Creation Script
# Run this script if you have a GitHub Personal Access Token

param(
    [Parameter(Mandatory=$false)]
    [string]$Token
)

if (-not $Token) {
    Write-Host "ERROR: GitHub Personal Access Token required" -ForegroundColor Red
    Write-Host ""
    Write-Host "Usage: .\create-github-repo.ps1 -Token YOUR_TOKEN" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "To create a token:" -ForegroundColor Cyan
    Write-Host "1. Go to: https://github.com/settings/tokens/new"
    Write-Host "2. Name: 'Physical AI Textbook Deploy'"
    Write-Host "3. Expiration: 90 days"
    Write-Host "4. Scopes: Select 'repo' (full control of private repositories)"
    Write-Host "5. Click 'Generate token' and copy it"
    Write-Host "6. Run: .\create-github-repo.ps1 -Token YOUR_TOKEN_HERE"
    Write-Host ""
    exit 1
}

Write-Host "Creating GitHub repository..." -ForegroundColor Cyan

# API endpoint
$uri = "https://api.github.com/user/repos"

# Repository data
$body = @{
    name = "Physical-AI-Humanoid-Robotics"
    description = "AI-Native Interactive Textbook for Physical AI and Humanoid Robotics with RAG Chatbot"
    homepage = "https://muhammadz24.github.io/Physical-AI-Humanoid-Robotics/"
    private = $false
    has_issues = $true
    has_projects = $true
    has_wiki = $true
} | ConvertTo-Json

# Headers
$headers = @{
    Authorization = "Bearer $Token"
    Accept = "application/vnd.github+json"
}

try {
    # Create repository
    $response = Invoke-RestMethod -Uri $uri -Method Post -Headers $headers -Body $body -ContentType "application/json"

    Write-Host "✓ Repository created successfully!" -ForegroundColor Green
    Write-Host "  URL: $($response.html_url)" -ForegroundColor White

    # Add git remote
    Write-Host "`nAdding git remote..." -ForegroundColor Cyan
    git remote add origin https://github.com/muhammadz24/Physical-AI-Humanoid-Robotics.git 2>$null

    if ($LASTEXITCODE -ne 0) {
        Write-Host "  Remote already exists, updating..." -ForegroundColor Yellow
        git remote set-url origin https://github.com/muhammadz24/Physical-AI-Humanoid-Robotics.git
    }

    Write-Host "✓ Git remote configured" -ForegroundColor Green

    # Push to GitHub
    Write-Host "`nPushing code to GitHub..." -ForegroundColor Cyan
    git push -u origin main

    if ($LASTEXITCODE -eq 0) {
        Write-Host "✓ Code pushed successfully!" -ForegroundColor Green
        Write-Host ""
        Write-Host "═══════════════════════════════════════════════════════════" -ForegroundColor Cyan
        Write-Host "  DEPLOYMENT COMPLETE!" -ForegroundColor Green
        Write-Host "═══════════════════════════════════════════════════════════" -ForegroundColor Cyan
        Write-Host ""
        Write-Host "Repository: https://github.com/muhammadz24/Physical-AI-Humanoid-Robotics" -ForegroundColor White
        Write-Host ""
        Write-Host "Next Steps:" -ForegroundColor Yellow
        Write-Host "1. Go to: https://github.com/muhammadz24/Physical-AI-Humanoid-Robotics/settings/pages"
        Write-Host "2. Under 'Build and deployment', select Source: GitHub Actions"
        Write-Host "3. Wait ~2 minutes for deployment"
        Write-Host ""
        Write-Host "Your site will be live at:" -ForegroundColor Cyan
        Write-Host "https://muhammadz24.github.io/Physical-AI-Humanoid-Robotics/" -ForegroundColor Green
        Write-Host ""
    } else {
        Write-Host "✗ Push failed. You may need to authenticate:" -ForegroundColor Red
        Write-Host "  Run: git push -u origin main" -ForegroundColor Yellow
    }

} catch {
    Write-Host "✗ Error creating repository:" -ForegroundColor Red
    Write-Host "  $($_.Exception.Message)" -ForegroundColor Red

    if ($_.Exception.Message -match "already exists") {
        Write-Host "`nRepository already exists. Proceeding with git setup..." -ForegroundColor Yellow

        git remote add origin https://github.com/muhammadz24/Physical-AI-Humanoid-Robotics.git 2>$null
        if ($LASTEXITCODE -ne 0) {
            git remote set-url origin https://github.com/muhammadz24/Physical-AI-Humanoid-Robotics.git
        }

        Write-Host "Run this command to push:" -ForegroundColor Cyan
        Write-Host "  git push -u origin main" -ForegroundColor White
    }
}
