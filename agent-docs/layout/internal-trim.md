# decad file layout: internal packages, trim graph

Internal packages of the trim-graph modify route: the lifted graph, exact tag offsets, face meshing and the rebuild.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/anasurf/` | Exact plane tags with interval offsets, three-plane meets as enclosures and strict half-space tests. See trim-graph Component 2. |
