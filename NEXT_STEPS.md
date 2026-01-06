# NEXT STEPS: Deploy Your Hybrid App

Congratulations! Your Physical AI & Humanoid Robotics textbook application is ready for deployment. Follow these steps to get it live on the internet:

## 1. Create a GitHub Repository

Choose one of these options:

**Option A: Using GitHub CLI**
```bash
gh repo create physical-ai-humanoid-robotics-textbook --public --push
```

**Option B: Manual Setup**
1. Go to [https://github.com/new](https://github.com/new)
2. Create a new public repository named `physical-ai-humanoid-robotics-textbook`
3. Add the remote origin to your local repo:
```bash
git remote add origin https://github.com/YOUR_USERNAME/physical-ai-humanoid-robotics-textbook.git
git branch -M main
git push -u origin main
```

## 2. Deploy to Vercel

1. Go to [https://vercel.com](https://vercel.com)
2. Sign in and click "New Project"
3. Import your GitHub repository
4. When configuring the project, Vercel should automatically detect:
   - **Framework**: Docusaurus
   - **Root Directory**: (leave as default)
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`

## 3. Add Environment Variables

After importing your project in Vercel, go to Settings > Environment Variables and add these variables from your `DEPLOYMENT_GUIDE.md`:

- `DATABASE_URL` - Your Neon Postgres database connection string
- `QDRANT_URL` - Your Qdrant Cloud instance URL
- `QDRANT_API_KEY` - Your Qdrant Cloud API key
- `GEMINI_API_KEY` - Your Google Gemini API key
- `JWT_SECRET` - Secret key for JWT token signing

## 4. Complete the Setup

1. Once deployed, access your live application
2. Run the ingestion script to populate your Qdrant vector database with textbook content:
```bash
cd backend
python scripts/ingest.py
```

## 5. Verify the Deployment

Check that all features are working:
- Frontend loads correctly
- Chatbot responds to questions about the textbook content
- Authentication (signup/signin) works properly
- Urdu language support functions as expected

---

**Your hybrid application featuring Docusaurus frontend, FastAPI backend, Gemini-powered RAG chatbot, and multilingual support is now ready for the world!**

For detailed deployment information, refer to your `DEPLOYMENT_GUIDE.md` file.