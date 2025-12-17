# Vercel Deployment Guide (Secure)

## Quick Deploy (Recommended)

### Option 1: Deploy with .env.production File

1. **Create `.env.production` in root**:
```bash
# Copy backend/.env values here
DATABASE_URL=<your-neon-db-url>
GEMINI_API_KEY=<your-gemini-key>
JWT_SECRET_KEY=<your-jwt-secret>
QDRANT_URL=<your-qdrant-url>
QDRANT_API_KEY=<your-qdrant-key>
```
**********
2. **Deploy**:
```bash
npx vercel********** --prod
```

Vercel will automatically read `.env.production` and inject variables.

---

### Option 2: Manual CLI Setup (One-Time)

1. **Install and Link**:
```bash
npm install -g vercel
vercel login
vercel link
```

2. **Add Environment Variables** (Run each command, paste value when prompted):
```bash
vercel env add DATABASE_URL production
# Paste: 

vercel env add GEMINI_API_KEY production
# Paste: 

vercel env add JWT_SECRET_KEY production
# Paste: **********

vercel env add QDRANT_URL production
# Paste: https://392dbde0-ca94-4a84-9e55-bd3106dabebe.europe-west3-0.gcp.cloud.qdrant.io

vercel env add QDRANT_API_KEY production
# Paste: **********

vercel env add QDRANT_COLLECTION production
# Paste: textbook_embeddings

vercel env add EMBEDDING_MODEL production
# Paste: sentence-transformers/all-MiniLM-L6-v2

vercel env add EMBEDDING_DIMENSION production
# Paste: 384

vercel env add GEMINI_MODEL production
# Paste: gemini-2.5-flash
```

3. **Deploy**:
```bash
vercel --prod
```

---

### Option 3: Automated PowerShell Script

Run `.\deploy_secure.ps1`:

```powershell
# deploy_secure.ps1
Write-Host "[INFO] Starting Secure Vercel Deployment..." -ForegroundColor Green

# Read .env file
$envFile = Get-Content backend\.env
$envVars = @{}

foreach ($line in $envFile) {
    if ($line -match "^([^#][^=]+)=(.+)$") {
        $key = $matches[1].Trim()
        $value = $matches[2].Trim()
        $envVars[$key] = $value
    }
}

# Deploy critical variables
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

foreach ($var in $criticalVars) {
    if ($envVars.ContainsKey($var)) {
        Write-Host "Setting $var..." -ForegroundColor Yellow
        $value = $envVars[$var]
        echo $value | vercel env add $var production
    }
}

Write-Host "[INFO] Deploying to production..." -ForegroundColor Green
vercel --prod

Write-Host "[SUCCESS] Deployment complete!" -ForegroundColor Green
```

---

## Verify Deployment

After deploy completes:

1. **Check URL**: Vercel will output production URL (e.g., `https://your-app.vercel.app`)
2. **Test Backend**: Visit `https://your-app.vercel.app/health`
3. **Test Frontend**: Visit `https://your-app.vercel.app`
4. **Check Env Vars**: Run `vercel env ls` to confirm all variables are set

---

## Troubleshooting

**Issue**: "vercel: command not found"
```bash
npm install -g vercel
```

**Issue**: Environment variables not working
```bash
vercel env pull .env.production  # Download from Vercel
cat .env.production  # Verify values
```

**Issue**: Database connection fails
- Verify DATABASE_URL includes `?sslmode=require`
- Check Neon database allows connections from Vercel IPs

---

## Security Notes

- ✅ `.env` files are in `.gitignore` (never committed)
- ✅ Environment variables stored securely in Vercel
- ✅ No secrets in chat history or code
- ✅ Production secrets separate from development

---

**NEXT STEP**: Run Option 1 (fastest) or Option 2 (most control)
