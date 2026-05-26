# Mechanical Design Guide for Aalto Space Association Projects with Fusion

## General

Keep up-to-date models always available to everyone (i.e. in Fusion) and remove obsolete models. Do not store up-to-date models personally, local copies are allowed for exploring ideas. If project specific instructions require design reviews, do not upload unreviewed models.

Use logical naming in English for all things (sketches, geometry, components, assemblies, features, parameters, etc.). Using logical naming conventions enables editing in the future.

Document all critical design decisions that have a major impact on the function of components as specified by project specific instructions.

When encountering problems or unknowns, first consult relevant documentation and ask for help from other project members. There is a great chance other's have encountered the same issues.

## Version Control

Use Fusion as the single source of truth. Always create a new version when making significant changes and include a short description. Do not overwrite reviewed designs without documentation. Only approved versions should be used for manufacturing or integration.

## Modeling

Always fully constrain sketches. All sketch entities (lines, curves. etc.) are dimensioned in reference to an origin or each other.

Models should always reflect real life geometry. Features that are supposed to fit together shall be toleranced. In additively manufactured parts (3D printing) use dimensions that have fit clearances etc. included in them. In other parts try to use whole numbers as nominal dimensions and add a tolerance in the drawing.

---

Keep all sketches simple. Separate design elements into their own sketches. This clearly communicates design intent and does not create unclear relations. Do not include chamfers, fillets or rounds in sketches. Start by adding constraints and add dimensions after them.

Build logical relationships between features using minimal references to ensure predictable changes. Use parent/child relationships intentionally and avoid unnecessary or fragile dependencies.

Avoid creating relationships to unstable features, such as:

- Faces and edges of chamfers and fillets
- Features in a pattern or mirror unless they are seed features
- Edges or seams of non-analytic surfaces (like curved loft surfaces)
- Generatively produced features (i.e. features made with geometry optimization, equations or externally)

Always fix all errors as they arise. If left unfixed they will cause problems later.

Use symmetry whenever possible: Design one portion (e.g., 1/4) and mirror it instead of recreating identical elements. Avoid redundant work: Repeating the same geometry is inefficient and increases the risk of misalignment or errors. You can use construction lines with symmetry constraints to dimension to full width on symmetric parts.

Add cosmetic features such as decals or extruded text, fillets, chamfers and rounds last after the main geometry.

Use the origin and the cardinal planes when possible. They should reflect the real-life function of the part (top plane is up and right plane is in the middle).

Do not take shortcuts. Edit features to make changes. (i.e. do not fill holes to remove them, or stack extrudes to make the part longer)

Use parametric modeling for all production parts. Parametric modeling defines parts with precise geometric relations and dimensions while keeping a logical timeline. As opposed to the other option, which is discouraged, is direct modeling which manipulates the geometry but doesn't always capture design history.

Use user-defined parameters for common dimensions to make updates easier and improve clarity, apply consistent naming.

Minimize suppressed features; avoid leaving unused junk in the timeline.

## Design for Manufacturing

Design for the manufacturing method. Additive manufacturing takes a different subset of skills as compared to milling or sheet metal modeling.

When designing free holes, make them larger than theoretically. Fine holes should be 5% larger than the nominal diameter, normal holes 10% larger and coarse holes 15% larger. In 3D printing holes shirk and should be validated for critical fits with test prints.

## Assemblies

1. Use folders for different sub-systems and use logical naming.
2. Create all components separately. Activate the current component that is being worked on.
3. Create sub-assemblies if they are logical. Large assemblies with flat trees are difficult to manage.
4. Keep the assembly aligned to it's origin. Figure out the most logical origin component.
5. Avoid cross-component references unless absolutely necessary and controlled.
6. The assembly should reflect the real-life state, obsolete components should be removed from the assembly model as well.
7. Run interference checks to ensure there are no unwanted fit or collision issues.
8. Remember to apply physical materials. This keeps COG and mass properties accurate and helps avoid mistakes.
9. Use skeleton modeling and avoid circular dependencies.

# Skeleton (Top-Down) Modeling (add visual guide for deriving parameters from skeletron here)

A skeleton model defines all critical dimensions and positions.  
Components reference the skeleton and not each other. The skeleton model defines where things are and components (parts and sub-assemblies) define the specific geometry.

1. Create a skeleton. Create a component "Skeleton_Master" in the main model which contains only:

- Sketches
- Planes, axes, points
- No solid bodies

2\. Define Parameters. Use Modify → Change Parameters where to add key dimensions (length, width, interface geometry)

3\. Build layout sketches. Create simple sketches on the origin plane that have key dimensions and interfaces (holes, positions). Fully constrain sketches

4\. Add Work Geometry

- Planes (offset, midplane)
- Axes (alignment)
- Points (holes, joints)

Use work geometry instead of referencing faces later

5\. Create Components. Model each part in its own component and use logical naming. Assemblies should be modeled as assemblies and not parts.

6\. References inside components:

Allowed:

- Skeleton sketches
- Skeleton work geometry
- User parameters

Project only the minimum required geometry from the skeleton.

7\. Position components via skeleton by aligning to the origin and skeleton planes and points.
