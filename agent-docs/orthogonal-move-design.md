# Orthogonal face move design

`MoveFace` is the press-pull of a CAD system for a faceted solid whose every face squares to a world axis, such as a part read from STL
or STEP and made a solid by `SolidFromMesh` (`agent-docs/mesh-solid-design.md`). It moves one planar face along its normal and lets
the walls beside it follow. The construction is the grid code of the orthogonal shell (`agent-docs/orthogonal-shell-design.md`),
in `internal/orthoshell/press.go`; the root entry is `move_face.go`.

## 1. Contract

```go
func MoveFace(ctx context.Context, b *Body, sel FaceSelector, distance units.Value) (*Body, error)
```

| Decision | Rule |
|---|---|
| M1 | `b` MUST be a live faceted solid whose held vertices all carry a zero bound and every triangle of which squares to a world axis exactly, otherwise `ErrUnsupported`; any other receiver keeps no route here. `sel` MUST resolve to exactly one face (`ErrDegenerate`, as a nil selector is `errNilSelector`). A nil context or body is `ErrDegenerate`. |
| M2 | `distance` is a signed length. A positive distance moves the face outward along its outward normal, growing the body, and a negative one inward, cutting it. Zero is `ErrDegenerate`, a non-length `ErrUnitKind`, and a non-finite value `ErrNotFinite`. |
| M3 | The face's triangles lie on one plane with one outward direction, and the grid is the solid's coordinates plus the plane moved to, `P + sign·distance`, a float sum. The face's footprint is its cells where its triangles cover more than half. |
| M4 | The result is the body joined to the prism of cells the face sweeps over its footprint (outward) or less it (inward). A prism that would enter material outward, or leave the solid inward (out through the opposite wall, or past a notch), or a move that leaves no solid, is `ErrDegenerate`: the face would pass through another part. |
| M5 | The mesh of the result's boundary is built as the shell's walls are, and read back by `solidFromMeshBody`, so coplanar triangles form faces by exact plane and every axis face keeps its plane. The result is committed as one `OperationMoveFace` entry with `b` consumed; a refusal leaves the document unchanged. |
| M6 | Moving a face keeps its neighbours' planes where the body is a polyhedron of axis faces: an outer wall moved outward lengthens the walls beside it, and a notch bottom moved in widens the notch's floor, because the swept prism is exactly the volume the face's motion adds or removes. |

## 2. Not covered

- A solid with a slanted or curved face, one with a vertex bound (a Boolean result), and a body that is not faceted.
- Several faces in one call: call `MoveFace` per face.
- Rotating a face, tapering the move, and a move that merges, splits or removes a face: delete-and-heal is not here.
