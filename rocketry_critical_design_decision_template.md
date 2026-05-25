# Engineering Decision Record (EDR) Template
## [EDR-XXXX]: [Descriptive Title of the Decision]

---

### Metadata
| Field | Details / Selection |
| :--- | :--- |
| **EDR ID** | EDR-XXXX *(e.g., EDR-PROP-001)* |
| **Status** | **[ Proposed | Under Review | Accepted | Rejected | Superseded ]** |
| **Subsystem** | **[ Propulsion | Avionics | Recovery | Structures | Payload | Operations ]** |
| **Author(s)** | [Name(s) and Role(s)] |
| **Date Created** | YYYY-MM-DD |
| **Date Resolved**| YYYY-MM-DD |
| **Impact Level** | **[ High | Medium | Low ]** *(High impacts budget, timeline, or vehicle safety)* |

---

## 1. Context and Problem Statement
*Provide a clear, concise description of the engineering problem or design choice that needs to be resolved. Why are we making this decision right now? Specify any relevant constraints (budget, lead time, fabrication capabilities, material availability, or competition rules like IREC/Spaceport America Cup).*

> **Example:** *We need to select the primary recovery deployment method for our 10k ft COTS solid motor rocket. The airframe must safely house the recovery components and ensure a descent velocity under 20 ft/s while minimizing drift.*

## 2. Design Requirements & Criteria
*List the hard constraints (Must-Haves) and evaluation criteria (Nice-to-Haves) used to judge the options. Whenever possible, make these quantitative.*

1. **Safety/Reliability:** Must have a failure probability below X% or include full redundancy.
2. **Mass Budget:** Total subsystem mass cannot exceed XXX grams/kg.
3. **Complexity:** Can be fabricated and assembled using in-house team tools and skills.
4. **Cost:** Total implementation cost must fit within the subsystem's $XXX budget.
5. **Timeline:** Must be designed, built, and tested by [Date].

## 3. The Decision / Selected Solution
*State the chosen design path clearly and unambiguously. Provide a technical overview of what is being implemented. Include architectural sketches, block diagrams, or CAD screenshots if applicable.*

> **Selected Path:** **[Name of Chosen Option]**

### Technical Summary
*Describe how the selected option will be implemented, its interface with other subsystems, and any specific materials/components selected.*

---

## 4. Alternatives Considered
*Detail the other design choices that were evaluated. For each alternative, provide a brief description and list its pros, cons, and why it was ultimately rejected.*

### Alternative A: [Name of Alternative A]
*Brief description of what this approach entails.*
* **Pros:**
  * Benefit 1 (e.g., Low weight)
  * Benefit 2 (e.g., Low cost)
* **Cons:**
  * Drawback 1 (e.g., High mechanical complexity)
  * Drawback 2 (e.g., Single point of failure)
* **Reason for Rejection:** *Why did this lose to the selected solution? Connect it back to the requirements in Section 2.*

### Alternative B: [Name of Alternative B]
*Brief description of what this approach entails.*
* **Pros:**
  * ...
* **Cons:**
  * ...
* **Reason for Rejection:** *...*

---

## 5. Rationale for the Selection
*Explain **why** the chosen solution was selected over the alternatives. Map its advantages directly back to the Design Requirements & Criteria outlined in Section 2.*

| Criteria | Selected Option | Alternative A | Alternative B |
| :--- | :---: | :---: | :---: |
| **Reliability** | ⭐⭐⭐ (High) | ⭐⭐ (Med) | ⭐ (Low) |
| **Mass Efficiency**| ⭐⭐ (Med) | ⭐⭐⭐ (High) | ⭐ (Low) |
| **Cost** | ⭐⭐ (Med) | ⭐ (High Cost)| ⭐⭐⭐ (Low Cost) |
| **Complexity** | Minimally complex | Extremely high | Moderately high |
| **Verdict** | **SELECTED** | **REJECTED** | **REJECTED** |

*Provide a narrative summary of the trade-offs made:*
> **Justification Narrative:** *While Alternative A was lighter, its manufacturing complexity posed an unacceptable risk to our launch timeline. The selected option strikes the best balance between reliability and ease of manufacturing, keeping us safely within our mass budget.*

---

## 6. Consequences, Trade-offs, and Risks
*Every decision has a cost. What are the negative side effects or new risks introduced by this choice? How will they be mitigated?*

* **Trade-off:** [e.g., Choosing a heavier material increases structural safety margins but decreases our maximum altitude (apogee).]
* **Identified Risk & Mitigation:**
  * **Risk:** [e.g., Lead time for the selected COTS component is 6 weeks.]
  * **Mitigation:** [e.g., Place the order immediately upon EDR approval; establish a secondary vendor pipeline.]

---

## 7. Implementation & Verification Plan
*How will the team execute this decision? Outline the next steps, manufacturing methods, and testing required to verify that the decision successfully solves the problem.*

* [ ] **Action Item 1:** Update the master CAD model to reflect this change. (Owner: [Name], Due: [Date])
* [ ] **Action Item 2:** Order long-lead components. (Owner: [Name], Due: [Date])
* [ ] **Testing/Verification:** *Detail how you will test this choice (e.g., static fire, deployment electronic e-match tests, vacuum chamber test, structural load test).*

---

## 8. References & Appendices
*Attach or link any supporting documentation used to make this decision.*
* **Simulations:** [Link to OpenRocket / RockSim / ANSYS results]
* **Data Sheets:** [Link to component specifications]
* **Prior Art:** [Link to past team documentation or research papers]

---

## 9. Sign-off & Approvals
*Ensures the leadership team and safety officers review and approve the choice before resources are spent.*

* **Subsystem Lead:** `[ Signature / Date ]`
* **Project Manager / Team Lead:** `[ Signature / Date ]`
* **Safety Officer:** `[ Signature / Date ]`
