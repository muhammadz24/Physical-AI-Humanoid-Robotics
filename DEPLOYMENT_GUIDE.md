# Deployment Guide: Physical AI & Humanoid Robotics Textbook

## Application Architecture

This is a **Hybrid Application** consisting of:

- **Frontend**: Docusaurus-based static site (React) for the interactive textbook
- **Backend**: FastAPI Python serverless functions for RAG chatbot functionality
- **Deployment**: Vercel serverless platform with unified frontend + backend

## Deployment Prerequisites

Before deploying, ensure you have:

1. A Vercel account
2. Access to the following external services:
   - Neon Postgres database
   - Qdrant vector database
   - Google Gemini API

## Environment Variables (Secrets)

Add these environment variables in your Vercel dashboard under Settings > Environment Variables:

### Required Secrets:

- `DATABASE_URL` - Your Neon Postgres database connection string
  - Format: `postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require`

- `QDRANT_URL` - Your Qdrant Cloud instance URL
  - Format: `https://xxx.qdrant.io`

- `QDRANT_API_KEY` - Your Qdrant Cloud API key

- `GEMINI_API_KEY` - Your Google Gemini API key

- `JWT_SECRET` - Secret key for JWT token signing
  - Generate a strong random secret (e.g., 32+ character random string)

## Vercel Build Settings

### Build Command:
```
npm run build
```

### Output Directory:
```
build
```

### Root Directory Settings:
- Do NOT change the root directory
- The project is configured to deploy both frontend and backend from the same repository

## Deployment Steps

1. **Push your code to a Git repository** (GitHub, GitLab, or Bitbucket)

2. **Import your project to Vercel**:
   - Go to Vercel Dashboard
   - Click "Add New Project"
   - Import your repository

3. **Configure Environment Variables**:
   - Go to Settings > Environment Variables
   - Add all required secrets listed above

4. **Deploy**:
   - Click "Deploy" and wait for the build to complete
   - Vercel will automatically detect the Docusaurus frontend and Python backend

5. **Post-Deployment Setup**:
   - After the initial deployment, you'll need to populate the Qdrant vector database
   - Run the ingestion script to populate textbook content:
     ```bash
     cd backend
     python scripts/ingest.py
     ```

## Architecture Notes

- The frontend (Docusaurus) serves static content from the `/build` directory
- Backend API routes are served via serverless functions at `/api/*`
- The `vercel.json` file handles routing between frontend and backend
- API requests to `/api/chat`, `/api/auth/*`, etc. are routed to the FastAPI backend
- All other requests are handled by the static Docusaurus frontend

## Troubleshooting

### Common Issues:

- **405 Method Not Allowed**: Check that `backend/main.py` has `redirect_slashes=False`
- **Database Connection Issues**: Verify `DATABASE_URL` includes `?sslmode=require`
- **Qdrant Connection Timeout**: Ensure `QDRANT_URL` and `QDRANT_API_KEY` are correct
- **500 Internal Server Error**: Check that all required environment variables are set

### Verification Steps:

1. Frontend loads at the root URL
2. API endpoints like `/api/health` return 200 status
3. Chat functionality works with textbook content
4. Authentication flows work properly

## Scaling Considerations

- Neon Postgres free tier has connection and compute limits
- Qdrant Cloud free tier may have request limits
- Vercel usage depends on traffic volume