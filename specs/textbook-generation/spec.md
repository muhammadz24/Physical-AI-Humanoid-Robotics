# Feature Specification: Textbook Generation

**Feature Branch**: `textbook-generation`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Define a complete, unambiguous specification for building the AI-native textbook with RAG chatbot"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Read Educational Content (Priority: P1)

As a learner, I want to access well-structured textbook chapters on Physical AI and Humanoid Robotics so I can learn concepts progressively from fundamentals to advanced topics.

**Why this priority**: This is the core value proposition - delivering educational content. Without readable content, no other features matter.

**Independent Test**: Can be fully tested by navigating to the deployed site, opening any chapter, and verifying content is readable, well-formatted, and complete. Delivers immediate educational value.

**Acceptance Scenarios**:

1. **Given** I am on the textbook homepage, **When** I navigate to Chapter 1 (Introduction to Physical AI), **Then** I see a well-formatted page with introduction, concepts, examples, and self-assessment questions
2. **Given** I am reading Chapter 3 (ROS 2 Fundamentals), **When** I scroll through the content, **Then** I see code examples with syntax highlighting, copy buttons, and line numbers
3. **Given** I am viewing any chapter, **When** I use the sidebar navigation, **Then** I can jump to any section within the chapter or switch to another chapter
4. **Given** I am reading on a mobile device, **When** I access any chapter, **Then** the content is fully responsive and readable without horizontal scrolling
5. **Given** I complete reading a chapter, **When** I reach the end, **Then** I see self-assessment questions and links to further resources

---

### User Story 2 - Ask Questions via RAG Chatbot (Priority: P2)

As a learner, I want to ask questions about the textbook content and receive accurate answers based exclusively on what I've been learning so I can clarify concepts without leaving the platform.

**Why this priority**: Enhances learning by providing instant clarification. Depends on P1 (content must exist first). This is the AI-native differentiator.

**Independent Test**: Can be tested by opening the chatbot interface, typing sample questions from each chapter, and verifying answers are accurate and grounded in textbook content. Delivers interactive learning value.

**Acceptance Scenarios**:

1. **Given** I am reading Chapter 2 (Basics of Humanoid Robotics), **When** I click the chatbot button and ask "What are the key components of a humanoid robot?", **Then** I receive an answer with citations to specific chapter sections
2. **Given** I ask a question outside the textbook scope, **When** the chatbot processes my query, **Then** I receive a clear message: "This topic is not covered in the current textbook. Please refer to the Further Resources section."
3. **Given** I ask "Explain ROS 2 nodes", **When** the chatbot retrieves relevant chunks, **Then** the response includes information exclusively from Chapter 3 with proper context
4. **Given** the chatbot is processing my query, **When** more than 3 seconds pass, **Then** I see a loading indicator or timeout message
5. **Given** I receive a chatbot answer, **When** I review the response, **Then** I can see which chapter/section the information came from

---

### User Story 3 - Select Text and Ask AI (Priority: P3)

As a learner, I want to highlight any text in the textbook and immediately ask the AI to explain, expand, or clarify it so I can get context-aware help without retyping my question.

**Why this priority**: Improves UX for chatbot interaction. Depends on P1 (content) and P2 (chatbot). Nice-to-have enhancement.

**Independent Test**: Can be tested by selecting text in any chapter, clicking the "Ask AI" option, and verifying the chatbot receives the selected text as context. Delivers convenience value.

**Acceptance Scenarios**:

1. **Given** I am reading Chapter 4 (Digital Twin Simulation), **When** I select the text "Isaac Sim provides GPU-accelerated physics", **Then** a tooltip or button appears with "Ask AI about this"
2. **Given** I selected text and clicked "Ask AI", **When** the chatbot interface opens, **Then** the selected text is pre-filled as context and I can ask a follow-up question
3. **Given** I selected text from a code block, **When** I click "Ask AI", **Then** the chatbot recognizes it as code and provides relevant programming context
4. **Given** I am on a mobile device, **When** I select text, **Then** the "Ask AI" option is accessible via touch-friendly UI

---

### User Story 4 - Navigate with Auto-Generated Sidebar (Priority: P1)

As a learner, I want an automatically generated sidebar that shows all chapters and sections so I can quickly navigate to any topic without manually searching.

**Why this priority**: Critical for usability. Part of P1 core experience. Docusaurus provides this built-in.

**Independent Test**: Can be tested by verifying sidebar displays all 6 chapters with nested sections and clicking through verifies navigation. Delivers navigation value.

**Acceptance Scenarios**:

1. **Given** I am on any page, **When** I view the sidebar, **Then** I see all 6 chapters listed in order with expandable sections
2. **Given** I am reading Chapter 3, **When** I view the sidebar, **Then** the current chapter and section are visually highlighted
3. **Given** I click a chapter in the sidebar, **When** the page loads, **Then** I am taken to the introduction of that chapter
4. **Given** I am on mobile, **When** I click the menu icon, **Then** the sidebar appears as a slide-out drawer

---

### User Story 5 - Toggle Dark Mode (Priority: P3)

As a learner, I want to switch between light and dark themes so I can read comfortably in different lighting conditions.

**Why this priority**: Accessibility and comfort feature. Docusaurus provides built-in. Low implementation cost.

**Independent Test**: Can be tested by clicking the theme toggle and verifying all pages switch themes. Delivers comfort value.

**Acceptance Scenarios**:

1. **Given** I am viewing the textbook, **When** I click the dark mode toggle, **Then** all pages switch to dark theme with appropriate contrast
2. **Given** I have selected dark mode, **When** I refresh the page, **Then** my preference is persisted via localStorage
3. **Given** I switch themes, **When** viewing code blocks, **Then** syntax highlighting adapts to the current theme

---

### User Story 6 - Search Textbook Content (Priority: P2)

As a learner, I want to search across all chapters for keywords so I can quickly find specific topics or concepts.

**Why this priority**: Enhances discoverability. Docusaurus provides built-in local search or Algolia integration.

**Independent Test**: Can be tested by typing keywords in search box and verifying relevant results appear. Delivers discovery value.

**Acceptance Scenarios**:

1. **Given** I am on any page, **When** I type "ROS 2 nodes" in the search box, **Then** I see results from Chapter 3 with highlighted snippets
2. **Given** I search for "Gazebo simulation", **When** results appear, **Then** I can click to navigate directly to that section in Chapter 4
3. **Given** I search for a term that doesn't exist, **When** results are displayed, **Then** I see a "No results found" message

---

### User Story 7 - View Code Examples with Interactive Features (Priority: P1)

As a learner, I want to view code examples with syntax highlighting, line numbers, and copy buttons so I can easily understand and reuse the code.

**Why this priority**: Essential for technical education. Part of P1 core content experience.

**Independent Test**: Can be tested by viewing code blocks in any chapter and verifying syntax highlighting, line numbers, and copy functionality work. Delivers technical learning value.

**Acceptance Scenarios**:

1. **Given** I am viewing a Python code example in Chapter 6, **When** I hover over the code block, **Then** I see a "Copy" button in the top-right corner
2. **Given** I click the copy button, **When** the code is copied, **Then** I receive visual feedback (e.g., checkmark or "Copied!" message)
3. **Given** I am viewing a code block, **When** the page renders, **Then** I see line numbers and appropriate syntax highlighting for the language (Python, YAML, etc.)
4. **Given** the code example has specific lines to highlight, **When** I view the block, **Then** those lines are visually emphasized

---

### User Story 8 - Access Further Resources (Priority: P2)

As a learner, I want to see curated links to official documentation, tutorials, and community resources at the end of each chapter so I can deepen my understanding.

**Why this priority**: Supports continuous learning beyond the textbook. Complements core content.

**Independent Test**: Can be tested by navigating to end of each chapter and verifying links are present and functional. Delivers extended learning value.

**Acceptance Scenarios**:

1. **Given** I complete Chapter 2, **When** I scroll to the end, **Then** I see a "Further Resources" section with categorized links (Official Docs, Tutorials, Community)
2. **Given** I click a resource link, **When** it opens, **Then** it opens in a new tab without disrupting my reading session
3. **Given** I view the Further Resources section, **When** I review the links, **Then** each link has a brief description of what to expect

---

### User Story 9 - Switch to Urdu Translation (Priority: P4 - Optional)

As an Urdu-speaking learner, I want to toggle between English and Urdu versions of the textbook so I can learn in my preferred language.

**Why this priority**: Optional accessibility feature. Significant translation effort. Only if resources allow.

**Independent Test**: Can be tested by clicking language toggle and verifying Urdu content appears. Delivers language accessibility value.

**Acceptance Scenarios**:

1. **Given** I am viewing any chapter, **When** I click the Urdu toggle, **Then** the content switches to Urdu while maintaining formatting and structure
2. **Given** I have selected Urdu, **When** I ask the chatbot a question in Urdu, **Then** I receive an answer in Urdu based on Urdu content chunks
3. **Given** I switch to Urdu, **When** I view code examples, **Then** code remains in original language but comments/explanations are in Urdu

---

### User Story 10 - Personalize Learning Path (Priority: P4 - Optional)

As a learner, I want to mark chapters as complete and see my progress so I can track my learning journey.

**Why this priority**: Optional engagement feature. Requires localStorage or backend tracking. Phase 3 enhancement.

**Independent Test**: Can be tested by marking chapters complete and verifying progress persists across sessions. Delivers motivation value.

**Acceptance Scenarios**:

1. **Given** I complete reading a chapter, **When** I click "Mark as Complete", **Then** a checkmark appears next to that chapter in the sidebar
2. **Given** I have completed 3 out of 6 chapters, **When** I view the homepage, **Then** I see a progress indicator showing 50% completion
3. **Given** I marked chapters complete, **When** I return to the site later, **Then** my progress is persisted via localStorage

---

### Edge Cases

- **What happens when a user asks the chatbot a question in a language other than English/Urdu?**
  → Chatbot responds: "Please ask questions in English. Urdu support is available via the language toggle."

- **What happens when the RAG chatbot retrieves chunks with low confidence (<70% similarity)?**
  → Chatbot responds: "I found some related information, but I'm not confident it fully answers your question. Please check the [relevant chapter] or rephrase your question."

- **What happens when the user selects extremely long text (>500 words) for "Ask AI"?**
  → System truncates to first 300 words and shows: "Selection truncated to 300 words for optimal AI processing."

- **What happens when the Qdrant or Neon free-tier limits are exceeded?**
  → Chatbot displays: "RAG service temporarily unavailable. Please refer to textbook content directly or try again later."

- **What happens when a user visits the site on a very old browser (e.g., IE11)?**
  → Display a banner: "For the best experience, please use a modern browser (Chrome, Firefox, Safari, Edge)."

- **What happens when JavaScript is disabled?**
  → Core textbook content remains readable (static HTML). Chatbot and interactive features display: "JavaScript required for interactive features."

- **What happens when a chapter has broken images or missing diagrams?**
  → Display placeholder with: "[Image: {description}] - Coming soon" and log error for content team to fix.

- **What happens when the user copies code that includes special characters or unicode?**
  → Code is copied exactly as displayed, preserving all characters and formatting.

- **What happens when the FastAPI backend is down?**
  → Chatbot displays: "Chat service temporarily unavailable. Please try again in a few minutes."

- **What happens when a user tries to search before the search index is loaded?**
  → Show loading spinner with message: "Initializing search..."

## Requirements *(mandatory)*

### Functional Requirements

#### Content & Structure

- **FR-001**: System MUST provide 6 complete chapters covering: (1) Introduction to Physical AI, (2) Basics of Humanoid Robotics, (3) ROS 2 Fundamentals, (4) Digital Twin Simulation (Gazebo + Isaac), (5) Vision-Language-Action Systems, (6) Capstone: Simple AI-Robot Pipeline
- **FR-002**: Each chapter MUST include: Introduction (200-300 words), Conceptual Foundation (800-1200 words), Technical Details (1000-1500 words), Hands-On Examples (500-800 words), Self-Assessment (5-10 questions), Further Resources
- **FR-003**: All code examples MUST include syntax highlighting, line numbers, copy button, and inline comments
- **FR-004**: All code examples MUST be executable on Ubuntu 22.04 with ROS 2 Humble
- **FR-005**: Content MUST be authored in MDX format compatible with Docusaurus v3.x

#### Navigation & UI

- **FR-006**: System MUST provide auto-generated sidebar navigation from file/folder structure
- **FR-007**: Sidebar MUST highlight current chapter and section
- **FR-008**: System MUST support responsive design for mobile, tablet, and desktop (min-width: 320px)
- **FR-009**: System MUST provide dark mode toggle with theme persistence via localStorage
- **FR-010**: System MUST include breadcrumb navigation showing current location
- **FR-011**: System MUST support Docusaurus built-in search (local or Algolia DocSearch)

#### RAG Chatbot

- **FR-012**: System MUST provide a chatbot interface accessible from all pages (floating button or fixed panel)
- **FR-013**: Chatbot MUST retrieve answers exclusively from textbook content chunks stored in Qdrant vector database
- **FR-014**: Chatbot MUST use sentence-transformers (all-MiniLM-L6-v2) for embeddings (384 dimensions)
- **FR-015**: Chatbot MUST return answers within 3 seconds (p95) or display timeout message
- **FR-016**: Chatbot MUST cite source chapter/section for each answer
- **FR-017**: Chatbot MUST display fallback message when query is outside textbook scope or confidence is low (<70%)
- **FR-018**: System MUST chunk textbook content into 300-500 token segments for optimal retrieval
- **FR-019**: Chatbot MUST support top-k retrieval (k=3-5 relevant chunks per query)
- **FR-020**: System MUST persist chat history in browser session (sessionStorage) until tab is closed

#### Text Selection & AI Interaction

- **FR-021**: System MUST allow users to select text and trigger "Ask AI about this" action
- **FR-022**: Selected text (up to 300 words) MUST be passed as context to chatbot
- **FR-023**: "Ask AI" tooltip/button MUST appear within 300ms of text selection
- **FR-024**: Selected text MUST be preserved in chatbot input for user to add follow-up questions

#### Deployment & Performance

- **FR-025**: System MUST build successfully with Docusaurus v3.x in under 60 seconds
- **FR-026**: System MUST start local dev server in under 10 seconds
- **FR-027**: System MUST deploy to GitHub Pages or Vercel free tier
- **FR-028**: System MUST achieve Lighthouse Performance Score >90
- **FR-029**: System MUST lazy-load images and heavy components
- **FR-030**: System MUST use code splitting to minimize initial bundle size

#### Backend & Data

- **FR-031**: Backend MUST be built with FastAPI (Python 3.10+)
- **FR-032**: System MUST use Qdrant Cloud free tier for vector storage (<1000 vectors initially)
- **FR-033**: System MUST use Neon Postgres free tier for metadata (chapter IDs, section references, timestamps)
- **FR-034**: System MUST generate embeddings locally using Sentence Transformers (no paid APIs)
- **FR-035**: RAG indexing pipeline MUST complete full textbook re-indexing in under 5 minutes
- **FR-036**: Backend API MUST implement CORS policies for frontend domain
- **FR-037**: Backend API MUST implement basic rate limiting (e.g., 20 requests/minute per IP)

#### Optional Features (Phase 3)

- **FR-038 (Optional)**: System MAY provide Urdu translation toggle with content stored as separate .mdx files
- **FR-039 (Optional)**: System MAY track chapter completion progress via localStorage
- **FR-040 (Optional)**: System MAY display progress indicator (% chapters completed) on homepage
- **FR-041 (Optional)**: Chatbot MAY support Urdu queries if Urdu content is available

### Key Entities *(include if feature involves data)*

- **Chapter**: Represents a textbook chapter. Attributes: ID (1-6), Title, Slug, MDX file path, Estimated reading time, Prerequisites
- **Section**: Represents a section within a chapter. Attributes: Section ID, Parent Chapter ID, Title, Heading level (h2/h3), Content type (text/code/diagram)
- **Content Chunk**: Represents a segment of textbook content for RAG. Attributes: Chunk ID, Chapter ID, Section ID, Text content, Embedding vector (384 dims), Token count (300-500)
- **Chat Message**: Represents a user query or chatbot response. Attributes: Message ID, Session ID, Timestamp, Role (user/assistant), Content, Source chunks (if assistant response)
- **Metadata**: Represents chapter/section metadata stored in Neon. Attributes: Chapter ID, Section ID, Title, File path, Last updated timestamp

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 6 chapters are complete, reviewed for accuracy, and deployed to production
- **SC-002**: Users can navigate to any chapter and read content with no broken links or missing images
- **SC-003**: Lighthouse Performance Score is >90 for all pages
- **SC-004**: Docusaurus build completes in <60 seconds for full site
- **SC-005**: RAG chatbot responds to 90% of test queries with accurate, grounded answers within 3 seconds
- **SC-006**: Chatbot achieves >90% retrieval precision (relevant chunks retrieved / total chunks retrieved)
- **SC-007**: 100% of code examples execute successfully on Ubuntu 22.04 + ROS 2 Humble
- **SC-008**: Site is fully responsive on mobile (tested on iOS Safari and Android Chrome)
- **SC-009**: Dark mode toggle works correctly with theme persistence across sessions
- **SC-010**: Full textbook content (6 chapters) generates <1000 vector embeddings, staying within Qdrant free tier
- **SC-011**: FastAPI backend deploys successfully to Vercel or similar free-tier platform
- **SC-012**: Zero security vulnerabilities in `npm audit` and `pip check` at deployment
- **SC-013**: Users can select text and trigger "Ask AI" with <300ms latency
- **SC-014**: RAG re-indexing pipeline completes in <5 minutes when content is updated

### Business/Educational Metrics

- **SC-015**: 90% of learners can complete Chapter 1 self-assessment with >70% accuracy (validates content clarity)
- **SC-016**: Chatbot usage shows >50% of users ask at least one question during their session (validates AI-native value)
- **SC-017**: Average session time is >8 minutes (indicates engaged learning)
- **SC-018**: <5% bounce rate on landing page (indicates compelling first impression)

## Constraints *(mandatory)*

### Technical Constraints

- **TC-001**: Frontend MUST use Docusaurus v3.x (no other static site generators)
- **TC-002**: Backend MUST use FastAPI (no Flask, Django, or other frameworks)
- **TC-003**: Vector database MUST be Qdrant Cloud free tier (1GB storage, 1M vectors max)
- **TC-004**: Relational database MUST be Neon Postgres free tier (512MB storage)
- **TC-005**: Embeddings MUST use Sentence Transformers all-MiniLM-L6-v2 (384 dimensions, local/free)
- **TC-006**: No paid APIs allowed (OpenAI, Cohere, Anthropic, etc.)
- **TC-007**: Deployment MUST use GitHub Pages or Vercel free tier
- **TC-008**: Total embeddings MUST stay under 1000 vectors initially (expandable if needed)
- **TC-009**: Frontend bundle size MUST be <2MB gzipped
- **TC-010**: No GPU-dependent operations in production environment

### Content Constraints

- **TC-011**: All content MUST be original or properly attributed with licenses
- **TC-012**: Code examples MUST not include proprietary or sensitive information
- **TC-013**: All external links MUST be to authoritative sources (official docs, peer-reviewed papers)
- **TC-014**: Content MUST be accessible (WCAG 2.1 AA minimum)

### Operational Constraints

- **TC-015**: Build process MUST complete on GitHub Actions free tier (2000 minutes/month)
- **TC-016**: Development MUST not require paid IDE or tooling
- **TC-017**: Local development MUST work on Ubuntu 22.04, macOS 12+, Windows 10+ (WSL2)

## Out of Scope *(mandatory)*

- **OOS-001**: User authentication and accounts (textbook is publicly accessible)
- **OOS-002**: Backend database for user-generated content (comments, reviews, etc.)
- **OOS-003**: Integration with LMS platforms (Moodle, Canvas, etc.)
- **OOS-004**: Video content or interactive simulations (text/code/images only)
- **OOS-005**: Real-time collaboration features (shared notes, study groups)
- **OOS-006**: Mobile native apps (responsive web only)
- **OOS-007**: Multi-version support (single version, updates replace previous)
- **OOS-008**: A/B testing infrastructure
- **OOS-009**: Analytics beyond basic GitHub Pages/Vercel built-in
- **OOS-010**: Community forum or discussion board (link to external Discord/Slack if needed)

## Dependencies *(mandatory)*

### External Dependencies

- **DEP-001**: Docusaurus v3.x (MIT License)
- **DEP-002**: React 18.x (MIT License)
- **DEP-003**: FastAPI (MIT License)
- **DEP-004**: Qdrant Cloud free tier account (API access)
- **DEP-005**: Neon Postgres free tier account (connection string)
- **DEP-006**: Sentence Transformers Python library (Apache 2.0)
- **DEP-007**: Node.js 18.x or 20.x (for Docusaurus build)
- **DEP-008**: Python 3.10+ (for FastAPI backend)
- **DEP-009**: GitHub account (for repository and Pages deployment)
- **DEP-010**: Vercel account (optional, if not using GitHub Pages)

### Content Dependencies

- **DEP-011**: Official ROS 2 documentation (https://docs.ros.org)
- **DEP-012**: Gazebo documentation (https://gazebosim.org/docs)
- **DEP-013**: NVIDIA Isaac Sim documentation (https://docs.omniverse.nvidia.com/isaacsim)
- **DEP-014**: Access to Physical AI & Humanoid Robotics course materials (for content creation)

### Development Dependencies

- **DEP-015**: Git (version control)
- **DEP-016**: Code editor (VS Code recommended, not required)
- **DEP-017**: npm or yarn (package management)
- **DEP-018**: pip (Python package management)

## Risks & Mitigations *(mandatory)*

### Risk 1: Qdrant Free Tier Limit Exceeded

**Probability**: Medium
**Impact**: High (chatbot unavailable)
**Mitigation**:
- Monitor vector count closely (target <800 initially to leave buffer)
- Optimize chunking strategy to reduce total chunks (e.g., larger chunks 400-500 tokens)
- If limit approached, consolidate similar chunks or increase chunk size
- Fallback: Temporarily disable chatbot with clear message, upgrade to paid tier if budget allows

### Risk 2: Neon Free Tier Database Limit Exceeded

**Probability**: Low
**Impact**: Medium (metadata unavailable, but content still readable)
**Mitigation**:
- Minimize metadata storage (only essential: chapter ID, section ID, title, path)
- Use lightweight schema with indexed lookups
- Fallback: Store metadata in static JSON files bundled with frontend

### Risk 3: Content Quality and Accuracy Issues

**Probability**: Medium
**Impact**: High (poor learning outcomes)
**Mitigation**:
- Implement peer review process for each chapter before merge
- Cross-reference all technical claims with official documentation
- Include subject matter expert (SME) review for Chapters 3-5
- Add errata page for user-reported issues
- Implement content versioning in git with clear changelog

### Risk 4: RAG Chatbot Hallucinations

**Probability**: Medium
**Impact**: High (misleading learners)
**Mitigation**:
- Enforce strict prompt engineering: "Answer ONLY based on provided context"
- Implement confidence threshold (>70%) and fallback message for low-confidence queries
- Show source chunks/citations with every answer for user verification
- Regular testing with adversarial queries
- User feedback mechanism to report incorrect answers

### Risk 5: Slow Build Times (Exceeds 60s Target)

**Probability**: Low
**Impact**: Medium (developer experience degradation)
**Mitigation**:
- Use Docusaurus incremental builds during development
- Optimize image sizes and lazy-load heavy assets
- Monitor build times in CI/CD and set alerts at 45s
- Profile builds to identify bottlenecks

### Risk 6: GitHub Pages Deployment Failures

**Probability**: Low
**Impact**: Medium (production unavailable)
**Mitigation**:
- Test deployment pipeline in staging branch first
- Use Docusaurus official deployment docs (https://docusaurus.io/docs/deployment)
- Have Vercel as backup deployment option
- Maintain rollback capability via git tags

### Risk 7: Backend Hosting Costs if Vercel Free Tier Exceeded

**Probability**: Low
**Impact**: Medium (unexpected costs or service interruption)
**Mitigation**:
- Monitor Vercel usage dashboard regularly
- Implement aggressive caching on API responses (60s TTL for embeddings)
- Set up alerts at 80% of free-tier bandwidth/function invocations
- Fallback: Migrate to Railway, Render, or self-hosted option if needed

### Risk 8: Browser Compatibility Issues

**Probability**: Low
**Impact**: Low (affects small user segment)
**Mitigation**:
- Test on modern browsers (Chrome, Firefox, Safari, Edge)
- Use Babel/Webpack polyfills for older browser support
- Display browser compatibility banner for outdated browsers

### Risk 9: Content Becomes Outdated (ROS 2, Gazebo, Isaac updates)

**Probability**: High
**Impact**: Medium (accuracy degrades over time)
**Mitigation**:
- Include "Last Updated" dates on each chapter
- Link to official documentation for latest details
- Schedule quarterly content review cycles
- Use version-specific references (e.g., "ROS 2 Humble" not just "ROS 2")

### Risk 10: Text Selection Feature Conflicts with Native Browser Behavior

**Probability**: Low
**Impact**: Low (UX annoyance)
**Mitigation**:
- Use non-intrusive tooltip or floating button for "Ask AI"
- Allow users to disable feature via settings/localStorage
- Test across devices and browsers for edge cases

## Open Questions *(to be resolved during planning)*

1. **Chatbot UI Placement**: Fixed panel on right side, floating button, or modal overlay? (Requires UX decision, see Docusaurus best practices)
2. **Search Provider**: Use Docusaurus local search or integrate Algolia DocSearch (free for open-source projects)? (Algolia recommended for better UX)
3. **Content Licensing**: What license should the textbook content use? (Suggest CC BY-SA 4.0 for educational content)
4. **Urdu Translation Scope**: If implemented, full translation or key concepts only? (Determine based on resources available)
5. **Code Playground Integration**: Should we embed RunKit, Repl.it, or similar for live code execution? (Out of scope for MVP, consider Phase 3)
6. **Diagram Tool**: Use Mermaid.js (Docusaurus built-in) or external tool like Excalidraw? (Mermaid.js recommended for maintainability)
7. **Contributor Workflow**: Allow community contributions via PRs or keep content team-only? (Recommend PR-based with review process)
8. **Analytics**: Use GitHub Pages built-in or add Google Analytics (free)? (Minimal analytics, prefer privacy-friendly Plausible if any)
9. **Feedback Mechanism**: Include "Was this helpful?" thumbs up/down on each page? (Recommended for content quality tracking)
10. **Version Control for Content**: Use git tags/branches for versioning or single main branch only? (Single main branch for MVP, tag releases)

## Acceptance Checklist *(final validation)*

### Phase 1: Core MVP

- [ ] All 6 chapters written, reviewed, and merged to main branch
- [ ] Docusaurus project initialized with proper configuration
- [ ] Auto-generated sidebar shows all chapters and sections
- [ ] Dark mode toggle functional with persistence
- [ ] All code blocks have syntax highlighting, line numbers, copy button
- [ ] Mobile responsive design tested on iOS and Android
- [ ] Lighthouse Performance Score >90 on all pages
- [ ] Build completes in <60 seconds
- [ ] Deployed to GitHub Pages or Vercel successfully
- [ ] RAG chatbot backend deployed with FastAPI
- [ ] Qdrant vector database populated with <1000 embeddings
- [ ] Neon Postgres populated with chapter/section metadata
- [ ] Chatbot responds to test queries with >90% accuracy
- [ ] Chatbot response time <3 seconds (p95)
- [ ] Chatbot citations show source chapter/section
- [ ] Low-confidence queries show appropriate fallback message
- [ ] Search functionality works across all chapters
- [ ] All external links verified and functional
- [ ] No broken images or missing diagrams
- [ ] All code examples tested on Ubuntu 22.04 + ROS 2 Humble
- [ ] `npm audit` and `pip check` show zero high/critical vulnerabilities
- [ ] README.md and contributor documentation complete

### Phase 2: Enhanced UX

- [ ] Text selection "Ask AI" feature implemented
- [ ] Selected text passed as context to chatbot
- [ ] Chapter progress tracking via localStorage
- [ ] Progress indicator on homepage

### Phase 3: Optional Features

- [ ] Urdu translation toggle (if approved)
- [ ] Urdu content files created and reviewed
- [ ] Chatbot supports Urdu queries
- [ ] Personalized learning paths
- [ ] Interactive code playgrounds (if feasible)

---

**Next Steps**: Proceed to `/sp.plan` to design the technical architecture and implementation strategy.
