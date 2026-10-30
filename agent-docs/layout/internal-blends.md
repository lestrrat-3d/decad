# decad file layout: internal packages, edge blends

Internal packages of the edge blends that rewrite a mesh Boolean result: retained-tag blends, the cross hole's
saddle fillet and the oblique boss fillet. The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/obliqueboss/` | Recognizes a complete oblique boss junction, admits its junction fillet and builds the mesh with its boundary, volume and area proofs. See oblique-boss-fillet design. |
| `internal/retainedblend/` | Admits Table RR edges and tangent chains between retained faces, rewrites the held mesh into the blend, cuts or fillets an R7 saddle, refuses a strip that meets another facet, and bounds the area and volume change. See retained-edge-blend and retained-modify-reach designs. |
| `internal/saddlecanal/` | The fillet of a cross hole's mouth in the cross-bore pose: existence, rows, points and the interval cell proofs of regularity, embedding, chord bound and areas. See retained-edge-blend §4.10. |
