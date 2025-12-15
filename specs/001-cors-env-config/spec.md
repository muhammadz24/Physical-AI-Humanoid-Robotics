# Feature Specification: Environment-Driven CORS and API Configuration

**Feature Branch**: 001-cors-env-config
**Created**: 2025-12-11  
**Status**: Draft

## Overview

Implement environment-variable driven CORS origins and API URLs to comply with Constitution Principle IX: Zero-Edit Deployment. Eliminate manual code changes between local and production environments.

## User Scenarios

### P1: Local Development Works Without Configuration

Developer clones repository and runs npm start and python main.py without environment configuration. Application works immediately.

**Acceptance**:
- Fresh clone with no .env file works with localhost
- Backend defaults to localhost:3000 and 127.0.0.1:3000
- Chat widget connects without CORS errors

### P2: Production Uses Environment Variables  

DevOps sets ALLOWED_ORIGINS on backend platform. No code changes required.

**Acceptance**:
- Backend with ALLOWED_ORIGINS=https://myapp.vercel.app allows that origin
- Multiple origins supported (comma-separated)
- Unauthorized origins blocked

### P3: Frontend Adapts Automatically

Frontend uses correct API URL based on deployment without code changes.

**Acceptance**:
- Local: uses localhost:8000
- Production: uses REACT_APP_API_URL value
- No hardcoded production URLs in source

## Requirements

**Backend:**
- FR-001: Load CORS origins from ALLOWED_ORIGINS env var (comma-separated)
- FR-002: Default to localhost:3000,127.0.0.1:3000 when unset
- FR-003: NO wildcard origins with credentials=True
- FR-004: Trim whitespace, validate http/https protocol
- FR-005: Log allowed origins on startup

**Frontend:**
- FR-006: Load API URL from REACT_APP_API_URL
- FR-007: Default to localhost:8000 when unset
- FR-008: No hardcoded production URLs
- FR-009: Append /api/chat to base URL

**Documentation:**
- FR-010: .env.example with ALLOWED_ORIGINS examples
- FR-011: README documents all env vars

## Success Criteria

- SC-001: Local dev starts in under 2 minutes, no config needed
- SC-002: Production needs exactly 1 backend env var, 1 frontend env var
- SC-003: Zero code changes for local→staging→production
- SC-004: Chat works in both local and production without CORS errors

## Dependencies

- Constitution v1.2.0 Principle IX
- FastAPI CORSMiddleware
- Docusaurus/React environment variables
