# Physical AI & Humanoid Robotics — Essentials Constitution

<!--
SYNC IMPACT REPORT
==================
Version Change: 1.1.0 → 1.2.0
Date: 2025-12-11

Modified Principles:
- None

Added Sections:
- Principle IX: Zero-Edit Deployment Configuration

Removed Sections:
- None

Templates Requiring Updates:
✅ plan-template.md - Updated to include deployment config verification
✅ spec-template.md - Updated to require environment variable specifications
✅ tasks-template.md - Updated to include deployment config tasks

Follow-up TODOs:
- None
-->

## Core Principles

### I. Simplicity-First Design
**The textbook and platform must prioritize clarity and ease of understanding above all else.**

- Content written for learners with minimal robotics background
- UI must be clean, intuitive, and distraction-free
- Features introduced only when they serve clear pedagogical value
- No feature bloat: every component must justify its existence
- Code examples must be minimal, executable, and well-commented

### II. Accuracy & Source Fidelity
**Every piece of information must be technically accurate and traceable to authoritative sources.**

- All technical content verified against official documentation (ROS 2, Gazebo, Isaac Sim, etc.)
- RAG chatbot answers MUST be grounded exclusively in textbook content
- No hallucinations or external knowledge injection in chatbot responses
- Citations and references included where appropriate
- Regular technical review cycles for content validation

### III. Free-Tier Architecture (NON-NEGOTIABLE)
**The entire platform must operate within free-tier constraints of all services.**

- Qdrant Cloud: Free tier vector storage limits respected
- Neon Postgres: Free tier database limits respected
- Vercel/GitHub Pages: Free hosting constraints adhered to
- No paid API dependencies (OpenAI, etc.) for core functionality
- Lightweight embeddings: minimize storage and compute costs
- Efficient chunking and retrieval strategies to stay within limits

### IV. Docusaurus Best Practices Compliance (NON-NEGOTIABLE)
**All UI/UX patterns must strictly follow official Docusaurus documentation.**

Authoritative Source:
- **Official Docs**: https://docusaurus.io/docs
- All frontend implementation decisions verified against official documentation
- Use Docusaurus built-in features before custom solutions
- Follow recommended project structure, configuration, and plugin patterns
- Leverage Docusaurus theming system (no custom CSS frameworks)
- Adhere to MDX best practices for content authoring

Core Docusaurus Features to Utilize:
- Docs versioning (if multi-version support needed)
- Built-in search (Algolia DocSearch or local search)
- Dark mode (built-in theme toggle)
- Sidebar navigation (auto-generated from file structure)
- Code block features (syntax highlighting, line numbers, copy button)
- Admonitions (:::note, :::tip, :::warning, :::danger)
- Tabs and interactive elements (built-in components)

### V. Minimalism in Technology Stack
**Use the smallest viable set of technologies; avoid over-engineering.**

Required Stack:
- **Frontend**: Docusaurus v3.x (static site generation, official best practices)
- **Backend**: FastAPI (lightweight Python API)
- **Vector DB**: Qdrant Cloud (free tier)
- **Database**: Neon Postgres (free tier)
- **Embeddings**: Sentence Transformers (local, open-source, all-MiniLM-L6-v2)
- **Deployment**: GitHub Pages or Vercel (free tier)

Forbidden:
- Heavy frameworks requiring significant build times
- Paid APIs or services
- GPU-dependent operations in production
- Complex microservices architectures
- Unnecessary abstraction layers
- Custom CSS frameworks (use Docusaurus theming)

### VI. Fast Build & Iteration Cycles
**Development, build, and deployment processes must be optimized for speed.**

- Docusaurus builds complete in under 60 seconds
- Local development server starts in under 10 seconds
- RAG indexing pipeline completes in under 5 minutes for full textbook
- CI/CD pipeline (if implemented) completes in under 10 minutes
- Hot-reload during development for instant feedback

### VII. Content-First Development
**Content quality and pedagogical value drive all technical decisions.**

- 6 chapters finalized before advanced features
- Each chapter must include:
  - Clear learning objectives
  - Conceptual explanations with diagrams
  - Hands-on code examples
  - Self-assessment questions
  - Further reading resources
- Content review precedes UI/UX enhancements
- Chatbot quality measured by answer accuracy, not feature count

### VIII. RAG Chatbot Guardrails
**The RAG chatbot must be helpful, accurate, and scope-limited.**

Core Requirements:
- Answers derived ONLY from textbook content chunks
- Clear indication when query is outside textbook scope
- Contextual retrieval (top-k relevant chunks with semantic search)
- Fallback message when confidence is low
- Optional features (Urdu translation, personalization) do not compromise accuracy

Quality Metrics:
- Answer relevance: >90% based on retrieval precision
- Response time: <3 seconds for query → answer
- No hallucinations: verifiable against source text

### IX. Zero-Edit Deployment Configuration (NON-NEGOTIABLE)
**All environment-specific configuration MUST use environment variables; code must deploy to any environment without manual edits.**

Backend Configuration Requirements:
- CORS origins MUST NOT be hardcoded
- Local development origins (`http://localhost:3000`, `http://127.0.0.1:3000`) hardcoded as defaults
- Production origin loaded from `ALLOWED_ORIGIN` environment variable
- Logic pattern (Python/FastAPI):
  ```python
  import os
  origins = ["http://localhost:3000", "http://127.0.0.1:3000"]
  prod_origin = os.getenv("ALLOWED_ORIGIN")
  if prod_origin:
      origins.append(prod_origin)
  ```
- Database URLs, API keys, and service endpoints MUST use environment variables
- No manual code changes required between local/staging/production deployments

Frontend Configuration Requirements:
- Backend API URL MUST use environment-aware configuration
- Pattern (Docusaurus/React):
  - Use `process.env.REACT_APP_API_URL` (or framework equivalent)
  - Fallback to `http://localhost:8000` for local development
  - Never hardcode production URLs in source code
- Build-time environment variables for static generation
- Runtime environment variables for dynamic client-side config (if applicable)

Deployment Documentation Requirements:
- README MUST document all required environment variables
- Example `.env.example` file provided with placeholders
- Platform-specific deployment guides (Vercel, Railway, etc.) with env var setup
- Clear separation between:
  - **Local Development**: Uses hardcoded localhost defaults
  - **Production**: Uses environment variables set on hosting platform

Rationale:
- Enables CI/CD without code modifications
- Supports multiple deployment environments (staging, production)
- Prevents accidental hardcoding of sensitive credentials
- Reduces deployment errors and manual intervention
- Aligns with twelve-factor app methodology

## Content Standards

### Chapter Structure Requirements
Each of the 6 chapters must follow this template:

1. **Introduction** (200-300 words)
   - Chapter objectives
   - Real-world relevance
   - Prerequisites

2. **Conceptual Foundation** (800-1200 words)
   - Core concepts explained simply
   - Analogies and visual aids
   - Historical context where relevant

3. **Technical Details** (1000-1500 words)
   - Technical specifications
   - Architecture diagrams
   - System interactions

4. **Hands-On Examples** (500-800 words)
   - Executable code snippets
   - Step-by-step walkthroughs
   - Common pitfalls and solutions

5. **Self-Assessment** (5-10 questions)
   - Multiple choice or short answer
   - Answers provided separately

6. **Further Resources**
   - Official documentation links
   - Recommended tutorials
   - Community resources

### Code Quality Standards
All code examples must:
- Run successfully on standard Ubuntu 22.04 / ROS 2 Humble setup
- Include inline comments explaining key concepts
- Follow PEP 8 (Python) or ROS 2 style guidelines
- Be self-contained or clearly document dependencies
- Include expected output or behavior description

## Performance Standards

### Frontend Performance
- Lighthouse Performance Score: >90
- First Contentful Paint: <1.5s
- Time to Interactive: <3.5s
- Cumulative Layout Shift: <0.1
- Lazy-loading for images and heavy components

### Backend Performance
- RAG query processing: <2s (p95)
- Vector search retrieval: <500ms (p95)
- Embedding generation (if on-demand): <1s per chunk
- API endpoint response times: <1s (p95)

### Database Efficiency
- Vector embeddings: max 384 dimensions (sentence-transformers/all-MiniLM-L6-v2)
- Chunk size: 300-500 tokens optimal for retrieval
- Total chunks for 6 chapters: <1000 estimated
- Metadata storage: minimal (chapter ID, section, page reference)

## Security & Privacy

### Data Handling
- No user authentication required for basic textbook access
- Optional chatbot usage tracking: anonymous, aggregated only
- No PII collection without explicit consent
- API rate limiting to prevent abuse (if public)
- CORS policies properly configured per Principle IX

### Dependency Security
- Regular `npm audit` and `pip check` for vulnerabilities
- Minimal third-party dependencies
- Pin dependency versions in lock files
- Automated security scanning in CI/CD (if implemented)

## Deployment & Operations

### Deployment Workflow
1. Content updates merged to `main` branch
2. Automated build triggered (GitHub Actions or Vercel)
3. Static site generated and deployed to GitHub Pages/Vercel
4. RAG re-indexing triggered if content changed
5. Smoke tests verify chatbot functionality
6. **Environment variables verified on hosting platform** (per Principle IX)

### Monitoring & Observability
- GitHub Pages/Vercel analytics for traffic
- Basic API logging (request count, latency, errors)
- Error tracking for chatbot failures
- No complex APM tools (stay within free tier)

### Rollback Strategy
- Git-based rollback for content issues
- Previous deployment artifacts retained
- Database migrations (if any) must be reversible
- Environment variable rollback if configuration errors occur

## Feature Roadmap Prioritization

### Phase 1: Core MVP (Must-Have)
1. Docusaurus textbook with 6 chapters
2. Basic RAG chatbot integration
3. Clean, responsive UI
4. GitHub Pages deployment
5. **Zero-edit deployment configuration** (Principle IX compliance)

### Phase 2: Enhanced UX (Nice-to-Have)
1. Select-text → Ask AI functionality
2. Chapter progress tracking (localStorage)
3. Dark mode toggle
4. Search functionality (Docusaurus built-in)

### Phase 3: Advanced Features (Optional)
1. Urdu translation toggle
2. Personalized learning paths
3. Interactive code playgrounds (if feasible in free tier)
4. Community Q&A integration

**Constraint**: Phase 2 features only after Phase 1 is fully validated. Phase 3 only if free-tier budget allows.

## Quality Gates

### Before Content Merge
- [ ] Technical accuracy verified by subject matter expert
- [ ] Code examples tested and executable
- [ ] Markdown formatting validated
- [ ] No broken links or missing images
- [ ] Passes spell-check and grammar review

### Before Feature Deployment
- [ ] Builds successfully in clean environment
- [ ] Chatbot answers verified against 10+ test queries
- [ ] Lighthouse performance score >90
- [ ] Mobile responsiveness tested
- [ ] Free-tier resource limits checked
- [ ] **Environment variables documented in README** (Principle IX)
- [ ] **No hardcoded production URLs/credentials in code** (Principle IX)

### Before Production Release
- [ ] All 6 chapters complete and reviewed
- [ ] RAG chatbot accuracy >90% on sample queries
- [ ] Deployment pipeline tested end-to-end
- [ ] Documentation for contributors complete
- [ ] License and attribution files in place
- [ ] **`.env.example` file provided with all required variables** (Principle IX)
- [ ] **Platform-specific deployment guides complete** (Principle IX)

## Governance

### Constitution Authority
- This constitution supersedes all ad-hoc technical decisions
- All code, content, and architectural changes must align with these principles
- Deviations require documented justification and approval

### Amendment Process
1. Proposed change documented with rationale
2. Impact assessment on existing content/features
3. Approval from project lead required
4. Migration plan for affected components
5. Version increment and changelog update

### Compliance Verification
- All pull requests reviewed for constitutional compliance
- Regular audits of free-tier resource usage
- Content quality reviews quarterly
- Technical debt tracked and addressed proactively
- **Deployment configuration audits** (verify environment variable usage, Principle IX)

### Living Document
- This constitution evolves with the project
- Learnings from user feedback incorporated
- Performance benchmarks adjusted based on real-world data
- Simplicity principle never compromised

**Version**: 1.2.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-11
