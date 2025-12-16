# Quickstart: Content Personalization

**Feature**: 007-content-personalization
**Branch**: `007-content-personalization`
**Date**: 2025-12-16

## Overview

This guide helps you set up, test, and integrate the content personalization feature into the Physical AI & Humanoid Robotics project. Personalization rewrites chapter content using Gemini LLM to match the user's experience level (beginner, intermediate, or expert).

## Prerequisites

Before starting, ensure:

1. **Feature 006 (Authentication) is complete**: Users must be authenticated to personalize content
2. **Backend is running**: FastAPI server on `http://localhost:8000`
3. **Database is accessible**: Neon Postgres with users table
4. **Gemini API key configured**: `GEMINI_API_KEY` in `backend/.env`
5. **User has experience levels set**: `software_experience` and `hardware_experience` fields in database

## Development Setup

### 1. Environment Configuration

**Backend `.env` file** (`backend/.env`):

```bash
# Database
DATABASE_URL=postgresql://user:password@host/database

# Authentication (from Feature 006)
JWT_SECRET_KEY=your_jwt_secret_here

# LLM Service
GEMINI_API_KEY=your_gemini_api_key_here
```

**Frontend (optional for production)** - Create `.env.local` in root:

```bash
# Production API URL (optional - defaults to localhost:8000)
REACT_APP_API_URL=https://your-api-domain.com
```

### 2. Install Dependencies

**Backend**:

```bash
cd backend
pip install -r requirements.txt
```

Verify personalization dependencies:
- `google-generativeai` (Gemini SDK)
- `python-jose[cryptography]` (JWT)
- `asyncpg` (Database)

**Frontend**:

```bash
npm install
```

No new frontend dependencies required - uses existing Docusaurus and React.

### 3. Start Services

**Terminal 1 - Backend**:

```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

Verify at: http://localhost:8000/docs (FastAPI Swagger UI)

**Terminal 2 - Frontend**:

```bash
npm start
```

Verify at: http://localhost:3000

## Testing the Personalization Endpoint

### Manual API Testing (curl)

**Step 1: Authenticate and get JWT cookie**:

```bash
curl -X POST http://localhost:8000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "testpassword"
  }' \
  -c cookies.txt
```

**Step 2: Personalize content**:

```bash
curl -X POST http://localhost:8000/api/personalize \
  -H "Content-Type: application/json" \
  -b cookies.txt \
  -d '{
    "chapter_text": "# Introduction to ROS 2\n\nROS 2 (Robot Operating System 2) is the next generation of the Robot Operating System, designed to meet the needs of modern robotics applications. It provides libraries and tools to help software developers create robot applications. ROS 2 builds on the success of ROS 1 while addressing its limitations in terms of real-time performance, security, and multi-platform support.",
    "chapter_id": "test-chapter-1"
  }'
```

**Expected Response** (beginner user example):

```json
{
  "personalized_text": "# Introduction to ROS 2 (Beginner-Friendly)\n\nROS 2 is like a communication system for robots. Think of it as a postal service that helps different parts of a robot talk to each other...",
  "original_length": 385,
  "personalized_length": 450,
  "processing_time_ms": 3200,
  "user_experience": {
    "software_experience": "beginner",
    "hardware_experience": "none"
  },
  "generated_at": "2025-12-16T10:30:45.123Z"
}
```

### FastAPI Swagger UI Testing

1. Navigate to http://localhost:8000/docs
2. Click "Authorize" and enter JWT token from cookie
3. Find `POST /api/personalize` endpoint
4. Click "Try it out"
5. Enter test payload:

```json
{
  "chapter_text": "# Test Chapter\n\nThis is a test chapter with at least 100 characters to meet validation requirements. ROS 2 is an advanced robotics middleware.",
  "chapter_id": "test-chapter"
}
```

6. Click "Execute" and verify 200 response

### Frontend Testing

**Step 1: Create test user with experience levels**:

1. Navigate to http://localhost:3000/signup
2. Fill in:
   - Name: Test User
   - Email: test@example.com
   - Password: testpassword123
   - Software Experience: **Beginner**
   - Hardware Experience: **None**
3. Click "Sign Up"

**Step 2: Test personalization button** (after implementation):

1. Navigate to a chapter page (e.g., Chapter 1)
2. Click "Personalize This Chapter" button
3. Observe loading state (spinner + "Personalizing content...")
4. Verify personalized content appears (simplified language)
5. Click "Show Original" to toggle back
6. Click "Show Personalized" to toggle forward

## Adding PersonalizeButton to Chapters

### Integration Steps

**1. Import PersonalizeButton in MDX file** (`docs/chapters/chapter-1.mdx`):

```jsx
---
id: chapter-1
title: Introduction to ROS 2
---

import PersonalizeButton from '@site/src/components/PersonalizeButton';

<PersonalizeButton />

# Introduction to ROS 2

[Your chapter content here...]
```

**2. Component automatically**:
- Detects user authentication state
- Extracts chapter text from parent `<article>` element
- Sends to `/api/personalize` with JWT cookie
- Caches response in component state
- Provides toggle UI

### Customization (Optional)

**Override chapter ID**:

```jsx
<PersonalizeButton chapterId="custom-chapter-id" />
```

**Custom button text**:

```jsx
<PersonalizeButton buttonText="Simplify This Content" />
```

## Troubleshooting

### Issue 1: "Authentication required" (401 Error)

**Symptoms**: Personalization fails with 401 response

**Causes**:
- User not logged in
- JWT token expired (7-day expiration)
- Cookie not being sent (CORS issue)

**Solutions**:
1. Verify user is authenticated: Check navbar for "Logout" button
2. Clear cookies and log in again
3. Check browser console for CORS errors
4. Verify `credentials: 'include'` in fetch call

### Issue 2: "Rate limit exceeded" (429 Error)

**Symptoms**: Personalization fails after 10 requests

**Causes**:
- User exceeded 10 personalizations per hour

**Solutions**:
1. Wait for rate limit window to reset (1 hour)
2. Increase rate limit in backend (for testing only)
3. Use different user account

### Issue 3: "LLM timeout" (503 Error)

**Symptoms**: Personalization times out after 30 seconds

**Causes**:
- Chapter text too long (>20K words)
- Gemini API slow or unavailable
- Network connectivity issues

**Solutions**:
1. Reduce chapter length (split into sections)
2. Retry request
3. Check Gemini API status
4. Verify `GEMINI_API_KEY` is valid

### Issue 4: Personalized content empty or malformed

**Symptoms**: Response has empty `personalized_text` or broken markdown

**Causes**:
- LLM prompt not working correctly
- Gemini API returned invalid response
- Content extraction failed

**Solutions**:
1. Check backend logs for LLM prompt and response
2. Verify chapter text is valid markdown
3. Test with shorter, simpler chapter
4. Iterate on LLM prompt template

### Issue 5: Frontend PersonalizeButton not appearing

**Symptoms**: Button doesn't render on chapter page

**Causes**:
- Component not imported in MDX
- User not authenticated (button hidden)
- React component error

**Solutions**:
1. Verify import statement in MDX: `import PersonalizeButton from '@site/src/components/PersonalizeButton';`
2. Check browser console for errors
3. Verify component file exists at `src/components/PersonalizeButton/index.js`
4. Check AuthProvider is wrapping app in `src/theme/Root.js`

### Issue 6: Toggle not working (shows loading forever)

**Symptoms**: Content doesn't switch between original and personalized

**Causes**:
- State management bug in usePersonalization hook
- API response not cached correctly

**Solutions**:
1. Check browser console for errors
2. Verify API response has `personalized_text` field
3. Clear component state by refreshing page
4. Check React DevTools for component state

## Performance Benchmarks

**Expected Performance**:

| Metric | Target | Typical |
|--------|--------|---------|
| API response time (short chapter <3K words) | <5s | 2-4s |
| API response time (long chapter 5-8K words) | <10s | 5-8s |
| Toggle between original/personalized | <200ms | <100ms |
| Component mount time | <500ms | <200ms |

**Monitoring**:
- Check `processing_time_ms` in API response
- Use browser DevTools Performance tab
- Monitor Gemini API quota usage

## Testing Different Experience Levels

Create test users with various experience combinations:

**Beginner + No Hardware**:
```json
{
  "software_experience": "beginner",
  "hardware_experience": "none"
}
```
Expected: Very simple language, detailed explanations, analogies

**Intermediate + Arduino**:
```json
{
  "software_experience": "intermediate",
  "hardware_experience": "arduino"
}
```
Expected: Balanced technical depth, some assumptions

**Pro + Professional**:
```json
{
  "software_experience": "pro",
  "hardware_experience": "professional"
}
```
Expected: Concise, technical, implementation-focused

## Production Deployment Checklist

Before deploying to production:

- [ ] Set `REACT_APP_API_URL` environment variable in Vercel/hosting
- [ ] Verify `GEMINI_API_KEY` is set in production backend
- [ ] Test rate limiting is enforced (10 req/hr/user)
- [ ] Verify HTTPS is used for API calls
- [ ] Test with multiple user types (beginner, intermediate, pro)
- [ ] Monitor Gemini API quota usage
- [ ] Set up error logging for failed personalizations
- [ ] Test on mobile devices (responsive design)
- [ ] Verify authentication cookies work cross-origin (SameSite settings)

## Next Steps

After completing Feature 007 POC on Chapter 1:

1. **Extend to all chapters**: Add PersonalizeButton to Chapters 2-6
2. **Iterate on LLM prompt**: Collect user feedback and refine prompt template
3. **Add analytics**: Track personalization usage and success rate
4. **Consider caching**: Evaluate database persistence for personalized content
5. **Add feedback mechanism**: Thumbs up/down on personalization quality

## Resources

- **Specification**: [spec.md](./spec.md)
- **Implementation Plan**: [plan.md](./plan.md)
- **Data Model**: [data-model.md](./data-model.md)
- **API Contract**: [contracts/personalize-api.yaml](./contracts/personalize-api.yaml)
- **Gemini API Docs**: https://ai.google.dev/docs
- **Docusaurus MDX**: https://docusaurus.io/docs/markdown-features

## Support

For issues or questions:
1. Check troubleshooting section above
2. Review backend logs (`uvicorn` output)
3. Check browser console for frontend errors
4. Review Prompt History Records in `history/prompts/007-content-personalization/`
5. Consult implementation plan for architectural decisions
