# decad file layout: Contact packages

Internal packages for contact classification and manifold publication.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/contactruling/` | Publishes a contact manifold from the clearance kernel's exact ruling certificate. See contact-geometry §4.5. |
| `internal/contactruling/placed/` | Classifies a placed source cylinder against an exact planar body and publishes the ruling manifold. See contact-geometry §4.5. |
| `internal/coplanarcontact/` | The switch for exact coplanar facet contact classification carried by Boolean evaluation contexts. See coplanar-contact §3. |
