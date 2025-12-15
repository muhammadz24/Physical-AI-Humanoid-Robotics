# Research: Qdrant Client API Migration (v1.7.3 → v1.16.1)

**Feature**: 003-fix-qdrant-attribute-error
**Date**: 2025-12-15
**Researcher**: Claude Sonnet 4.5

## Executive Summary

**ROOT CAUSE IDENTIFIED**: The `search()` method was deprecated in qdrant-client v1.13.0 and completely removed in v1.16.0. The installed version (v1.16.1) no longer supports the `search()` API, causing the AttributeError.

**SOLUTION**: Migrate from `client.search()` to `client.query_points()` API.

## Research Findings

### 1. Qdrant Client API Breaking Changes

**Key Finding**: The `search()` method along with `recommend()`, `discover()`, and related methods were removed in v1.16.0 as part of a major API modernization.

**Affected Methods Removed in v1.16.0**:
- `search` → replaced by `query_points`
- `search_batch` → replaced by `query_batch_points`
- `recommend`, `recommend_batch` → replaced by `query_points` with different params
- `discover`, `discover_batch` → replaced by `query_points`
- `upload_records` → deprecated alternative APIs

**Migration Path**:
- Old API (v1.7.3 and earlier): `client.search()`
- New API (v1.16.1): `client.query_points()`

### 2. API Signature Comparison

**Old API (v1.7.3) - using `search()`:**

```python
results = client.search(
    collection_name="my_collection",
    query_vector=[0.1, 0.2, ...],  # parameter name: query_vector
    limit=5,
    score_threshold=0.7,
    query_filter=Filter(...),
    with_payload=True
)
```

**New API (v1.16.1) - using `query_points()`:**

```python
results = client.query_points(
    collection_name="my_collection",
    query=[0.1, 0.2, ...],  # parameter name changed: query (not query_vector)
    limit=5,
    score_threshold=0.7,
    query_filter=Filter(...),
    with_payload=True
)
```

**Key Differences**:
1. Method name: `search()` → `query_points()`
2. Parameter name: `query_vector` → `query`
3. Return type: Same structure (list of scored points)

### 3. Current Code Analysis

**File**: `backend/app/core/vector_store.py`
**Lines**: 127-134

**Current Implementation (BROKEN):**

```python
results = self.client.search(
    collection_name=self.collection_name,
    query_vector=query_vector,  # ❌ Old parameter name
    limit=top_k,
    score_threshold=score_threshold,
    query_filter=search_filter,
    with_payload=True
)
```

**Required Fix:**

```python
results = self.client.query_points(
    collection_name=self.collection_name,
    query=query_vector,  # ✅ New parameter name
    limit=top_k,
    score_threshold=score_threshold,
    query_filter=search_filter,
    with_payload=True
)
```

### 4. Version Reconciliation Decision

**Current State**:
- `requirements.txt`: specifies `qdrant-client==1.7.3` (old API)
- Installed: `qdrant-client==1.16.1` (new API)

**Decision**: Update to v1.16.1 and migrate code

**Rationale**:
1. v1.16.1 is already installed and tested in environment
2. v1.16.x includes important features: BM25 support, connection pooling, performance improvements
3. Downgrading to v1.7.3 is not recommended (outdated, missing features)
4. Code migration is straightforward (method rename + parameter rename)

**Risk Assessment**:
- LOW: API change is well-documented
- LOW: Migration requires only 2 lines of code change
- NONE: Return type and structure remain unchanged

### 5. Additional Qdrant Client Features in v1.16.1

**New Capabilities** (not needed for this fix, but worth noting):
- Built-in BM25 support (no fastembed dependency)
- GRPC connection pooling for better performance
- Server-side score boosting (v1.13.0+)
- Native inference support via Fastembed integration (v1.13.0+)
- Optional vectors config for collections without dense vectors (v1.13.0+)

**Backward Compatibility**:
- Existing stored vectors: ✅ Compatible (no data migration needed)
- Collection schema: ✅ Compatible
- Filter syntax: ✅ Compatible
- Payload structure: ✅ Compatible

## Recommendations

### Primary Recommendation: Migrate to query_points API

1. **Update `backend/app/core/vector_store.py`**:
   - Line 127: Change `self.client.search(` to `self.client.query_points(`
   - Line 129: Change `query_vector=query_vector` to `query=query_vector`

2. **Update `backend/requirements.txt`**:
   - Line 8: Change `qdrant-client==1.7.3` to `qdrant-client==1.16.1`

3. **No data migration required**: Existing vectors and collections remain compatible

### Alternative Considered: Downgrade to v1.7.3

**Rejected because**:
- Loses access to newer features (BM25, connection pooling)
- v1.7.3 is 9+ minor versions behind (potential security/stability issues)
- Code migration is minimal (2 lines)

## Testing Strategy

1. **Unit Test**: Verify `VectorStoreManager.search()` wrapper method works
2. **Integration Test**: POST `/api/chat` with sample query
3. **Verify**: No AttributeError in logs
4. **Verify**: Search results match expected structure

## References

- [Qdrant Client Python GitHub Releases](https://github.com/qdrant/qdrant-client/releases)
- [Qdrant Client v1.16.0 Release Notes](https://github.com/qdrant/qdrant-client/releases/tag/v1.16.0) - Removed deprecated methods
- [Qdrant Python Client Documentation](https://python-client.qdrant.tech/)
- [Qdrant Client PyPI Page](https://pypi.org/project/qdrant-client/)

## Conclusion

The AttributeError is caused by calling the removed `search()` method on qdrant-client v1.16.1. The fix requires migrating to the `query_points()` API, which is a straightforward 2-line code change. No data migration or schema changes are needed.

**Implementation Complexity**: LOW ✅
**Risk Level**: LOW ✅
**Estimated Implementation Time**: <10 minutes ✅
