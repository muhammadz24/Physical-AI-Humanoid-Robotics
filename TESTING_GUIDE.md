# Local Testing Guide - RAG Chatbot

**Status**: Ready for local testing
**Prerequisites**: Backend API keys configured

---

## Step-by-Step Testing Instructions

### Step 1: Start Backend Server

```bash
# Open Terminal 1
cd backend
python main.py
```

**Expected Output**:
```
🚀 Starting FastAPI backend...
✅ Database connection pool established
✅ Qdrant vector store connected
📥 Loading embedding model: sentence-transformers/all-MiniLM-L6-v2...
✅ Embedding model loaded successfully (dimension: 384)
✅ All services initialized successfully
INFO:     Uvicorn running on http://0.0.0.0:8000
```

**Verify Backend Health**:
```bash
# Open Terminal 2
curl http://localhost:8000/health
```

Should return:
```json
{
  "status": "healthy",
  "services": {
    "neon_postgres": "connected",
    "qdrant_vector_store": "connected"
  }
}
```

---

### Step 2: Start Frontend Server

```bash
# In project root (Terminal 3 or new terminal)
npm start
```

**Expected Output**:
```
[SUCCESS] Docusaurus website is running at:
  http://localhost:3000/
```

Browser will auto-open to http://localhost:3000

---

### Step 3: Test Chat Widget

**3.1 Verify Floating Button**:
- Look for circular chat button bottom-right corner
- Should be blue/primary color
- Hover should scale up slightly

**3.2 Open Chat**:
- Click floating button
- Chat modal should expand
- Header: "AI Assistant"
- Empty state message: "Ask me anything about the Physical AI & Humanoid Robotics textbook!"

**3.3 Send Query**:
- Type: "What is ROS 2?"
- Press Enter or click send button
- Loading indicator appears (3 bouncing dots)
- Bot response displays with answer
- Citations appear as clickable chips below answer

**3.4 Click Citation**:
- Click any citation chip
- Should navigate to corresponding chapter/section
- Chat modal closes

---

### Step 4: Test Select-to-Ask

**4.1 Select Text**:
- Navigate to any chapter page
- Select text (e.g., "Unitree G1" or "Isaac Sim")
- Selection must be 5+ characters

**4.2 Tooltip Appears**:
- Small tooltip appears near selected text
- Button says: "Ask AI about this"

**4.3 Ask AI**:
- Click tooltip button
- Chat opens automatically
- Query pre-filled: "Explain: [your selected text]"
- Query auto-submits
- Response displays with context

---

### Step 5: Test Dark Mode

- Click moon icon in navbar (top-right)
- Chat background should change to dark
- Text should remain readable
- Citations should have dark mode colors

---

### Step 6: Test Mobile Layout

**Desktop Browser Method**:
- Open browser DevTools (F12)
- Click device toolbar icon (Ctrl+Shift+M)
- Select iPhone or Android device
- Refresh page

**Expected**:
- Chat goes full-screen when opened
- Floating button smaller (56px)
- All features functional

---

## Example Queries

### Query 1: Basic Question
```
What is ROS 2?
```
**Expected**: Answer from Chapter 3 with citations

### Query 2: Technical Topic
```
How does SLAM work with Isaac ROS?
```
**Expected**: Answer from Chapter 9

### Query 3: Hardware Specific
```
Explain the Unitree G1 humanoid robot
```
**Expected**: Answer from Chapters 11-13

### Query 4: Out of Context
```
What is the weather today?
```
**Expected**: "I don't have enough information in the textbook to answer this question"

---

## Troubleshooting

### Issue: "Backend connection error"

**Cause**: FastAPI server not running or wrong port

**Fix**:
1. Check Terminal 1 shows "Uvicorn running on http://0.0.0.0:8000"
2. Test: `curl http://localhost:8000/health`
3. Check backend/.env has correct settings

---

### Issue: "No results found"

**Cause**: Qdrant has no data (ingestion not run)

**Fix**:
```bash
cd backend
python scripts/ingest.py
```

---

### Issue: "OpenAI API error"

**Cause**: Invalid or missing OpenAI API key

**Fix**:
1. Get API key from https://platform.openai.com
2. Add to backend/.env: `OPENAI_API_KEY=sk-...`
3. Restart backend server

---

### Issue: Chat widget not appearing

**Cause**: Frontend not built correctly

**Fix**:
```bash
npm install
npm start
```

---

### Issue: Citations not clickable

**Cause**: URL format issue in backend

**Fix**: Check backend logs for citation URL format

---

## Manual Testing Checklist

### Core Features
- [ ] Floating button visible
- [ ] Chat opens/closes
- [ ] Messages send and display
- [ ] Loading indicator works
- [ ] Citations render as chips
- [ ] Citations are clickable
- [ ] Navigation works from citations

### Select-to-Ask
- [ ] Tooltip appears on text selection
- [ ] Tooltip button clickable
- [ ] Chat opens with pre-filled query
- [ ] Query auto-submits
- [ ] Context passed to backend

### UI/UX
- [ ] Dark mode toggle works
- [ ] Mobile layout responsive
- [ ] Animations smooth
- [ ] Scrolling smooth
- [ ] Input focus works
- [ ] Enter key sends message

### Error Handling
- [ ] Backend down shows error message
- [ ] Empty query disabled
- [ ] API errors handled gracefully
- [ ] No console errors

---

## Success Criteria

✅ Backend starts without errors
✅ Frontend loads at localhost:3000
✅ Chat widget appears on all pages
✅ Can send queries and receive responses
✅ Citations clickable and navigate correctly
✅ Select-to-ask functional
✅ Dark mode works
✅ Mobile layout responsive
✅ No JavaScript errors in console

---

## Next Steps After Testing

1. **If all tests pass**: Notify user - ready for git commit
2. **If tests fail**: Debug and fix issues
3. **After user approval**: Commit to git
4. **Production deployment**: Deploy to GitHub Pages + Railway

---

**Testing Duration**: ~15-20 minutes for complete manual test
**Required**: Both servers running simultaneously
