# Container Design

A `Container` is a named group inside one `Document`. It holds sketches, planes and bodies and
nests inside other containers. It is bookkeeping only: it has no transform, it instances nothing,
and it changes no geometry. `agent-docs/api-design.md` §13 keeps assemblies (`Component` and
`Occurrence` instancing and the graph that comes with it) a non-goal. A container is not that. A
consumer that maps a model onto a tool with its own component tree reads the grouping from here.

## 1. API

```go
func (d *Document) Root() *Container
func (c *Container) NewChild(name string) (*Container, error)
func (c *Container) Name() string
func (c *Container) Parent() *Container        // nil for the root
func (c *Container) Children() []*Container    // creation order
func (c *Container) AddSketch(s *sketch.Sketch) error
func (c *Container) AddPlane(p *sketch.Plane) error
func (c *Container) Sketches() []*sketch.Sketch  // order of AddSketch
func (c *Container) Planes() []*sketch.Plane      // order of AddPlane
func (c *Container) Bodies() []*Body              // live bodies, creation order
func (d *Document) MoveBody(b *Body, to *Container) error
func (d *Document) ContainerOf(b *Body) (*Container, error)
func (d *Document) ContainerOfSketch(s *sketch.Sketch) *Container
```

- Every document has one root container, named `""`. A body, sketch or plane that was never placed
  belongs to the root.
- `NewChild` takes a non-empty name that is unique among its siblings; anything else is
  `ErrDegenerate`.
- A container belongs to one document for life. There is no delete or rename.
- `AddSketch` and `AddPlane` place the sketch or plane in `c`, and take it out of the container it was
  in. Adding it again to `c` changes nothing. The document keeps the link by the sketch's or plane's
  pointer, so `sketch` needs no change.
- `Bodies` lists the live bodies placed directly in `c`, not those of its children. A retired body
  leaves its container with the live set.
- A nil argument is `ErrDegenerate`.

## 2. Where a result lands

Placement is decided where an operation commits, from its inputs, never from a global setting.

| Operation | Result lands in |
|---|---|
| `Union`, `Cut`, `Intersect`, `Combine`, `Hole`, `Fillet`, `Chamfer`, `Shell`, `Draft`, `Thicken`, `Offset`, `Transform`, `Mirrored`, `ScaledAt`, `Patterned`, `PatternJoin`, `PatchEdges`, `Stitch`, `Trim`, `Extend`, `Split`, `Unstitch` | the container of its first (target) body |
| `Copy`, `MirroredCopy`, `PatternCopies` | the container of the source body; every copy lands there |
| `Extrude`, `Revolve`, `Sweep`, `Coil`, `PatchProfile`, `Loft`, `LoftChain` | the container of its sketch (of the first sketch for a loft) |
| `LoftSections`, `LoftBetween`, `LoftFromPoint` | the container of the first section's or end's sketch |
| `LoftFaces`, `LoftFacesToConeTangent`, `ConeFromFace` | the container of the body the first face belongs to |

An operation that commits many bodies puts every one of them in the same container.

## 3. Operands from different containers

An operation that reads more than one body or sketch, or a body and a sketch, refuses when they sit in
different containers (a boolean's operands, a loft's sections, a pattern join's target and feature,
an extent that names a body, `SweepPair`'s two bodies): the error is `ErrCrossContainer` (new sentinel, `errors.Is`) naming both
container paths. The check runs with the other operand checks, before any geometry work, and a
refused call leaves the document unchanged.

`MoveBody(b, to)` is the one way across. It puts a live body in `to` and nothing else: the body
keeps its handle, coordinates, provenance and measurements, no producer identity advances, and
`Verify` and `Bodies` are unchanged. A foreign body or container is `ErrForeignBody`, a retired body
`ErrRetiredBody`. A tool that allows a combine only inside one component builds in one container and
moves the finished body.

## 4. Concurrency and state

Container state is document state, guarded by the same lock as the live set. `Body` stays immutable:
its container is not a field of it. `Bodies`, `Remove`, `Verify` and the readers above may run
concurrently with one another; a mutation of the tree (`NewChild`, `AddSketch`, `AddPlane`,
`MoveBody`) is a mutation of the document and not safe to run concurrently with another.

## 5. What does not change

Geometry, measurements, `Verify`, export and every existing refusal keep their results. A document
that never uses a container behaves as before: every body, sketch and plane is in the root and no
check can fail.

## 6. Tests

| Test | What it shows |
|---|---|
| `apitest.TestContainerTree` | names, parents, children order, unique sibling names, foreign container |
| `apitest.TestContainerPlacement` | each row of §2 lands where the table says |
| `apitest.TestContainerCrossRefusal` | a boolean, a stitch and an extrude to a body in another container refuse with `ErrCrossContainer`, leaving the document unchanged |
| `apitest.TestContainerMoveBody` | `MoveBody` changes the container only; volume, bounds and `Verify` are unchanged |
