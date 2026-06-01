# RFC-XXX: <Descriptive Title>

**Author(s):** <Your Name(s)>  
**Created:** <YYYY-MM-DD>  
**Status:** Draft | In Review | Accepted | Rejected | Superseded  
**Supersedes:** <RFC-###> (optional)  
**Superseded by:** <RFC-###> (optional)

---

## 1. Summary

Briefly describe what this RFC proposes and *why* — one or two short paragraphs.  
This should be understandable at a glance, even by people not deep into the codebase.

---

## 2. Motivation

Explain the reasoning behind this proposal:
- What problem or limitation are you addressing?
- Why is this important or urgent now?
- Are there any related documents, issues, or other RFCs?

---

## 3. Goals

List the primary goals (and optionally, non-goals).

**Example:**
- Improve API performance for bulk data operations.
- Reduce coupling between modules.
- Avoid breaking existing API contracts.

---

## 4. Detailed Design

Describe the design or technical approach in detail.  
You can include:
- Architecture diagrams (embedded or linked images)
- Example data structures or class diagrams
- API contract changes (with before/after examples)
- Configuration or deployment changes

Use fenced code blocks where appropriate:

```ts
interface UserProfile {
  id: string;
  email: string;
  name?: string;
}
```

---

## 5. Alternatives Considered

List and evaluate any alternative approaches you considered and explain *why* you didn’t choose them.  
This adds helpful context for future readers.

---

## 6. Drawbacks

Be honest about potential downsides, risk factors, or complexity introduced.

---

## 7. Impact

Consider the impact on:
- Existing systems
- API consumers
- Infrastructure and cost
- Security/privacy
- Developer experience

---

## 8. Rollout Plan

Describe how this RFC will be implemented and rolled out:
- Phased migration plan
- Feature flags
- Monitoring / rollback strategy

---

## 9. References

Include any related materials:
- Links to related RFCs
- GitHub issues or PRs
- External documentation or specs
