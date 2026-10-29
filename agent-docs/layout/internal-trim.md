# decad file layout: internal packages, trim graph

Internal packages of the trim-graph modify route: the lifted graph, exact tag offsets, face meshing and the rebuild.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/anasurf/` | Exact plane tags with interval offsets, three-plane meets as enclosures and strict half-space tests. See trim-graph Component 2. |
| `internal/trimgraph/` | Lifts a retained faceted body into faces, edges and vertices on exact plane tags, and decides edge convexity from the tags. See trim-graph Component 1, Table L. |
| `internal/trimtest/` | Hand-made retained prisms and a pyramid with exact plane frames, for the trim-graph packages' tests. |
| `internal/facemesh/` | Meshes one planar face of a trim graph over `internal/triangulation`, refusing a dropped point or a triangle that fails the exact orientation margin. See trim-graph Component 4. |
| `internal/trimrebuild/` | Splices new faces into a receiver's held mesh, composes the volume, area and diameter bounds, runs the held-embedding gate and builds the body. See trim-graph Component 5. |
